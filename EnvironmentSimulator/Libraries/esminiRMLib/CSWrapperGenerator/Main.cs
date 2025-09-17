using ClangSharp.Interop;

namespace CppToCSharpWrapper
{
    class Program
    {
        static unsafe void Main(string[] args)
        {
            if (args.Length < 2)
            {
                Console.WriteLine("Usage: CppToCSharpWrapper <input.h> <output.cs>");
                return;
            }

            string headerFile = args[0];
            string outputFile = args[1];
            // string namespaceName = args.Length > 2 ? args[2] : "GeneratedBindings";
            // string dllName = args.Length > 3 ? args[3] : "library";
            string namespaceName = "OpenDRIVE";
            string dllName = "esminiRMLib";

            var generator = new WrapperGenerator(namespaceName, dllName);
            generator.GenerateWrapper(headerFile, outputFile);
        }
    }

    public enum TypeMappingContext
    {
        FunctionParameter,
        StructField
    }

    public unsafe class WrapperGenerator
    {
        private string namespaceName;
        private string dllName;
        private List<string> output = new List<string>();
        private Dictionary<string, StructInfo> structs = new Dictionary<string, StructInfo>();
        private HashSet<string> functions = new HashSet<string>();
        private HashSet<string> enums = new HashSet<string>();
        private HashSet<string> processedFiles = new HashSet<string>();
        private HashSet<string> referencedTypes = new HashSet<string>();
        private Dictionary<string, CXCursor> typeDefinitions = new Dictionary<string, CXCursor>();
        private List<CXCursor> mainFileDeclarationCursors = new List<CXCursor>();
        private List<CXTranslationUnit> additionalTranslationUnits = new List<CXTranslationUnit>(); // Keep additional TUs alive
        
        // Helper method to check if a type is declared in the main file
        private bool IsMainFileDeclaration(string typeName)
        {
            return mainFileDeclarationCursors.Any(cursor => cursor.Spelling.ToString() == typeName);
        }

        public WrapperGenerator(string namespaceName, string dllName)
        {
            this.namespaceName = namespaceName;
            this.dllName = dllName;
        }

        public void GenerateWrapper(string headerFile, string outputFile)
        {
            var index = CXIndex.Create();
            
            // Enhanced compiler arguments for better parsing
            var compilerArgs = new string[]
            {
                "-std=c++17",
                "-DWIN32",
                "-D_WIN32",
                "-I.",  // Include current directory
                "-I..", // Include parent directory
                "-fparse-all-comments",
                "-Wno-pragma-once-outside-header"
            };

            Console.WriteLine($"Parsing header file: {headerFile}");
            Console.WriteLine($"Compiler arguments: {string.Join(" ", compilerArgs)}");
            
            var translationUnit = CXTranslationUnit.Parse(index, headerFile, compilerArgs, 
                null, CXTranslationUnit_Flags.CXTranslationUnit_None);

            if (translationUnit.Handle == IntPtr.Zero)
            {
                Console.WriteLine("Failed to parse header file");
                return;
            }

            // Check for parsing diagnostics
            var numDiagnostics = translationUnit.NumDiagnostics;
            Console.WriteLine($"Parsing diagnostics: {numDiagnostics}");
            for (uint i = 0; i < numDiagnostics; i++)
            {
                var diagnostic = translationUnit.GetDiagnostic(i);
                Console.WriteLine($"  {diagnostic.Severity}: {diagnostic.Spelling}");
            }

            output.Add("/*");
            output.Add(" * esmini - Environment Simulator Minimalistic");
            output.Add(" * https://github.com/esmini/esmini");
            output.Add(" *");
            output.Add(" * This Source Code Form is subject to the terms of the Mozilla Public");
            output.Add(" * License, v. 2.0. If a copy of the MPL was not distributed with this");
            output.Add(" * file, You can obtain one at https://mozilla.org/MPL/2.0/.");
            output.Add(" *");
            output.Add(" * Copyright (c) partners of Simulation Scenarios");
            output.Add(" * https://sites.google.com/view/simulationscenarios");
            output.Add(" */");
            output.Add("");
            output.Add("/*");
            output.Add(" * This module provides a generic C# interface/wrapper to the RoadManagerDLL library");
            output.Add(" * simply mirroring the interface in terms of datatypes and functions");
            output.Add(" */");
            output.Add("");
            output.Add("");
            output.Add("using System;");
            output.Add("using System.Runtime.InteropServices;");
            output.Add("");
            output.Add($"namespace {namespaceName}");
            output.Add("{");

            var cursor = translationUnit.Cursor;

            // First pass: Process main file and collect referenced types
            Console.WriteLine("First pass: Processing main file and collecting referenced types...");
            CollectMainFileDeclarations(cursor);

            // Second pass: Process dependencies for referenced types
            Console.WriteLine("Second pass: Processing dependencies for referenced types...");
            ProcessReferencedDependencies(cursor);

            // Third pass: Generate the actual wrapper code
            Console.WriteLine("Third pass: Generating wrapper code...");

            // Additional pass: Analyze comments for roadmanager:: references and discover additional enums
            Console.WriteLine("Additional pass: Analyzing comments for roadmanager:: references...");
            var roadmanagerReferences = AnalyzeCommentsForRoadManagerTypes(cursor);
            DiscoverEnumsFromRoadManagerReferences(roadmanagerReferences, cursor);

            // Generate enums and structs (and other value types) from dependencies and main file directly under the namespace
            foreach (var typeName in referencedTypes)
            {
                if (typeDefinitions.TryGetValue(typeName, out var typeCursor))
                {
                    if (typeCursor.Kind == CXCursorKind.CXCursor_EnumDecl)
                    {
                        ProcessEnum(typeCursor, typeName);
                    }
                    else if (typeCursor.Kind == CXCursorKind.CXCursor_StructDecl)
                    {
                        ProcessStruct(typeCursor, typeName);
                    }
                }
            }
            foreach (var decl in mainFileDeclarationCursors)
            {
                if (!string.IsNullOrEmpty(decl.Spelling.ToString()))
                {
                    if (decl.Kind == CXCursorKind.CXCursor_EnumDecl)
                    {
                        ProcessEnum(decl, decl.Spelling.ToString());
                    }
                    else if (decl.Kind == CXCursorKind.CXCursor_StructDecl)
                    {
                        ProcessStruct(decl, decl.Spelling.ToString());
                    }
                }
            }

            // Open static class
            output.Add("    public static class RoadManagerLibraryCS");
            output.Add("    {");
            output.Add("#if UNITY_EDITOR_LINUX || UNITY_STANDALONE_LINUX");
            output.Add("        private const string LIB_NAME = \"libesminiRMLib.so\";");
            output.Add("#else");
            output.Add("        private const string LIB_NAME = \"esminiRMLib\";");
            output.Add("#endif");
            output.Add("");

            // Generate functions inside the static class
            foreach (var decl in mainFileDeclarationCursors)
            {
                if (decl.Kind == CXCursorKind.CXCursor_FunctionDecl && !string.IsNullOrEmpty(decl.Spelling.ToString()))
                {
                    ProcessFunction(decl, decl.Spelling.ToString());
                }
            }

            output.Add("    }"); // Close RoadManagerLibraryCS class
            output.Add("}"); // Close namespace

            File.WriteAllLines(outputFile, output);
            Console.WriteLine($"Generated wrapper: {outputFile}");
            Console.WriteLine($"Processed {structs.Count} structs, {functions.Count} functions from main file and dependencies");

            // Clean up additional translation units
            foreach (var tu in additionalTranslationUnits)
            {
                tu.Dispose();
            }
            additionalTranslationUnits.Clear();

            translationUnit.Dispose();
            index.Dispose();
        }

        private void CollectMainFileDeclarations(CXCursor cursor)
        {
            Console.WriteLine("Scanning main file for declarations...");
            int declarationCount = 0;
            
            cursor.VisitChildren((child, parent, clientData) =>
            {
                var kind = child.Kind;
                var spelling = child.Spelling.ToString();
                var location = child.Location;
                var isFromMainFile = location.IsFromMainFile;
                
                // Debug: Show all top-level cursors
                if (parent.Kind == CXCursorKind.CXCursor_TranslationUnit)
                {
                    Console.WriteLine($"  Found cursor: {kind} '{spelling}' (MainFile: {isFromMainFile})");
                }

                if (isFromMainFile && !string.IsNullOrEmpty(spelling))
                {
                    if (kind == CXCursorKind.CXCursor_StructDecl ||
                        kind == CXCursorKind.CXCursor_FunctionDecl ||
                        kind == CXCursorKind.CXCursor_EnumDecl)
                    {
                        mainFileDeclarationCursors.Add(child);
                        declarationCount++;
                        Console.WriteLine($"    -> Added main file declaration: {kind} '{spelling}'");
                        
                        // Collect types referenced by this declaration
                        CollectReferencedTypes(child);
                    }
                }
                return CXChildVisitResult.CXChildVisit_Recurse;
            }, default(CXClientData));
            
            Console.WriteLine($"Found {declarationCount} main file declarations");
            Console.WriteLine($"Found {referencedTypes.Count} referenced types from dependencies");
        }

        private void CollectReferencedTypes(CXCursor cursor)
        {
            cursor.VisitChildren((child, parent, clientData) =>
            {
                var kind = child.Kind;
                
                // Collect types from function parameters, return types, and struct fields
                if (kind == CXCursorKind.CXCursor_ParmDecl || 
                    kind == CXCursorKind.CXCursor_FieldDecl)
                {
                    var type = child.Type;
                    CollectTypeFromCXType(type);
                }
                else if (kind == CXCursorKind.CXCursor_FunctionDecl)
                {
                    // Collect return type
                    var returnType = child.ResultType;
                    CollectTypeFromCXType(returnType);
                }
                
                return CXChildVisitResult.CXChildVisit_Recurse;
            }, default(CXClientData));
        }

        private void CollectTypeFromCXType(CXType type)
        {
            // Handle pointer types
            if (type.kind == CXTypeKind.CXType_Pointer)
            {
                CollectTypeFromCXType(type.PointeeType);
                return;
            }
            
            // Handle record types (structs, classes)
            if (type.kind == CXTypeKind.CXType_Record)
            {
                var typeName = type.Spelling.ToString();
                if (!string.IsNullOrEmpty(typeName) && !IsMainFileDeclaration(typeName))
                {
                    referencedTypes.Add(typeName);
                    
                    // Try to find the type definition
                    var typeDecl = type.Declaration;
                    if (typeDecl.Kind != CXCursorKind.CXCursor_NoDeclFound)
                    {
                        typeDefinitions[typeName] = typeDecl;
                    }
                }
            }
            
            // Handle enum types
            if (type.kind == CXTypeKind.CXType_Enum)
            {
                var typeName = type.Spelling.ToString();
                if (!string.IsNullOrEmpty(typeName) && !IsMainFileDeclaration(typeName))
                {
                    referencedTypes.Add(typeName);
                    
                    var typeDecl = type.Declaration;
                    if (typeDecl.Kind != CXCursorKind.CXCursor_NoDeclFound)
                    {
                        typeDefinitions[typeName] = typeDecl;
                    }
                }
            }
        }

        private void ProcessReferencedDependencies(CXCursor cursor)
        {
            // Process all the referenced types that were found in dependencies
            foreach (var typeName in referencedTypes)
            {
                if (typeDefinitions.TryGetValue(typeName, out var typeCursor))
                {
                    var location = typeCursor.Location;
                    location.GetFileLocation(out var filename, out _, out _, out _);
                    var filenameStr = filename.ToString();
                    
                    if (!string.IsNullOrEmpty(filenameStr) && !processedFiles.Contains(filenameStr))
                    {
                        Console.WriteLine($"Processing dependency type: {typeName} from {filenameStr}");
                        processedFiles.Add(filenameStr);
                    }
                }
            }
        }

        private void GenerateWrapperCode(CXCursor cursor)
        {
            Console.WriteLine("Generating wrapper code...");
            
            // First, generate code for dependency types (outside the static class)
            Console.WriteLine($"Generating {referencedTypes.Count} dependency types...");
            foreach (var typeName in referencedTypes)
            {
                if (typeDefinitions.TryGetValue(typeName, out var typeCursor))
                {
                    var kind = typeCursor.Kind;
                    Console.WriteLine($"  Generating dependency: {kind} '{typeName}'");
                    if (kind == CXCursorKind.CXCursor_StructDecl)
                    {
                        ProcessStruct(typeCursor, typeName);
                    }
                    else if (kind == CXCursorKind.CXCursor_EnumDecl)
                    {
                        ProcessEnum(typeCursor, typeName);
                    }
                }
            }
            
            // Generate structs and enums from main file (outside the static class)
            Console.WriteLine($"Generating main file types...");
            foreach (var decl in mainFileDeclarationCursors)
            {
                var kind = decl.Kind;
                var spelling = decl.Spelling.ToString();

                if (kind == CXCursorKind.CXCursor_StructDecl && !string.IsNullOrEmpty(spelling))
                {
                    Console.WriteLine($"  Processing main file struct: {spelling}");
                    ProcessStruct(decl, spelling);
                }
                else if (kind == CXCursorKind.CXCursor_EnumDecl && !string.IsNullOrEmpty(spelling))
                {
                    Console.WriteLine($"  Processing main file enum: {spelling}");
                    ProcessEnum(decl, spelling);
                }
            }
            
            // Generate functions inside the static class
            Console.WriteLine($"Generating main file functions...");
            foreach (var decl in mainFileDeclarationCursors)
            {
                var kind = decl.Kind;
                var spelling = decl.Spelling.ToString();

                if (kind == CXCursorKind.CXCursor_FunctionDecl && !string.IsNullOrEmpty(spelling))
                {
                    Console.WriteLine($"  Processing main file function: {spelling}");
                    ProcessFunction(decl, spelling);
                }
            }
        }

        private void ProcessCursor(CXCursor cursor)
        {
            cursor.VisitChildren((child, parent, clientData) =>
            {
                if (child.Location.IsFromMainFile)
                {
                    var kind = child.Kind;
                    var spelling = child.Spelling.ToString();

                    if (kind == CXCursorKind.CXCursor_StructDecl && !string.IsNullOrEmpty(spelling))
                    {
                        ProcessStruct(child, spelling);
                    }
                    else if (kind == CXCursorKind.CXCursor_FunctionDecl && !string.IsNullOrEmpty(spelling))
                    {
                        ProcessFunction(child, spelling);
                    }
                    else if (kind == CXCursorKind.CXCursor_EnumDecl && !string.IsNullOrEmpty(spelling))
                    {
                        ProcessEnum(child, spelling);
                    }
                }
                return CXChildVisitResult.CXChildVisit_Continue;
            }, default(CXClientData));
        }

        private void ProcessStruct(CXCursor cursor, string structName)
        {
            if (structs.ContainsKey(structName))
                return;

            // Skip unnamed/anonymous structs that can't be properly represented in C#
            if (string.IsNullOrEmpty(structName) || 
                structName.Contains("(unnamed struct") || 
                structName.Contains("(anonymous struct") ||
                structName.Contains("::") ||
                structName.StartsWith("(") ||
                !IsValidCSharpIdentifier(structName))
            {
                Console.WriteLine($"    Skipping unnamed or invalid struct: '{structName}'");
                return;
            }

            var cleanStructName = CleanTypeName(structName);
            var structInfo = new StructInfo { Name = cleanStructName };
            structs[structName] = structInfo;

            // Add comment indicating source file if it's from a dependency
            if (referencedTypes.Contains(structName))
            {
                var location = cursor.Location;
                location.GetFileLocation(out var filename, out _, out _, out _);
                var filenameStr = filename.ToString();
                if (!string.IsNullOrEmpty(filenameStr))
                {
                    var shortFilename = Path.GetFileName(filenameStr);
                    output.Add($"    // Struct from dependency: {shortFilename}");
                }
            }

            output.Add($"    [StructLayout(LayoutKind.Sequential), Serializable]");
            output.Add($"    public struct {cleanStructName}");
            output.Add("    {");

            cursor.VisitChildren((child, parent, clientData) =>
            {
                if (child.Kind == CXCursorKind.CXCursor_FieldDecl)
                {
                    var originalFieldName = child.Spelling.ToString();
                    var fieldName = ConvertSnakeCaseToCamelCase(originalFieldName);
                    fieldName = ApplySpecialCaseTransformations(fieldName);
                    var fieldType = MapType(child.Type, TypeMappingContext.StructField, out bool isByRef, out bool isReadOnly, out bool isOutParam);
                    
                    // Extract field comment
                    var fieldComment = ExtractFieldComment(child);
                    
                    // Add field comment if available
                    if (!string.IsNullOrWhiteSpace(fieldComment))
                    {
                        output.Add($"        /// <summary>{fieldComment}</summary>");
                    }
                    
                    if (isByRef)
                    {
                        if (isReadOnly)
                            output.Add($"        public readonly {fieldType} {fieldName};");
                        else
                            output.Add($"        public {fieldType} {fieldName};");
                    }
                    else
                    {
                        output.Add($"        public {fieldType} {fieldName};");
                    }
                    
                    structInfo.Fields.Add(new FieldInfo { Name = fieldName, Type = fieldType, Comment = fieldComment });
                }
                return CXChildVisitResult.CXChildVisit_Continue;
            }, default(CXClientData));

            output.Add("    }");
            output.Add("");
        }

        private void ProcessFunction(CXCursor cursor, string functionName)
        {
            Console.WriteLine($"    Processing function '{functionName}'");
            
            if (functions.Contains(functionName))
            {
                Console.WriteLine($"    Function '{functionName}' already processed, skipping");
                return;
            }

            functions.Add(functionName);

            // Check function linkage
            var linkage = cursor.Linkage;
            Console.WriteLine($"    Function linkage: {linkage}");
            
            var returnType = MapType(cursor.ResultType, TypeMappingContext.FunctionParameter, out _, out _, out _);
            Console.WriteLine($"    Return type: {cursor.ResultType.ToString()} -> {returnType}");
            
            var parameters = new List<string>();
            var paramCount = cursor.NumArguments;
            var usedParameterNames = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            Console.WriteLine($"    Parameter count: {paramCount}");

            for (uint i = 0; i < paramCount; i++)
            {
                var param = cursor.GetArgument(i);
                var paramName = param.Spelling.ToString();
                var paramType = MapType(param.Type, TypeMappingContext.FunctionParameter, out bool isByRef, out bool isReadOnly, out bool isOutParam);
                
                Console.WriteLine($"      Param {i}: {param.Type.ToString()} -> {paramType} {paramName}");
                
                if (string.IsNullOrEmpty(paramName))
                    paramName = $"param{i}";

                // Clean up parameter names and apply better naming conventions
                paramName = CleanParameterName(paramName, param.Type);
                
                // Ensure parameter name uniqueness
                paramName = EnsureUniqueParameterName(paramName, usedParameterNames);
                usedParameterNames.Add(paramName);
                
                if (isByRef)
                {
                    if (isOutParam)
                        parameters.Add($"out {paramType} {paramName}");
                    else if (isReadOnly)
                        parameters.Add($"in {paramType} {paramName}");
                    else
                        parameters.Add($"ref {paramType} {paramName}");
                }
                else
                {
                    parameters.Add($"{paramType} {paramName}");
                }
            }

            var paramStr = string.Join(", ", parameters);
            
            // Generate XML documentation comment
            output.Add($"        /// <summary>");
            output.Add($"        /// {GenerateFunctionDocumentation(functionName, cursor)}");
            output.Add($"        /// </summary>");
            
            // Add parameter documentation
            for (uint i = 0; i < paramCount; i++)
            {
                var param = cursor.GetArgument(i);
                var paramName = param.Spelling.ToString();
                if (string.IsNullOrEmpty(paramName))
                    paramName = $"param{i}";
                
                // Apply same cleaning and uniqueness logic for documentation
                var originalName = paramName;
                paramName = CleanParameterName(paramName, param.Type);
                var tempUsedNames = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
                for (uint j = 0; j < i; j++)
                {
                    var prevParam = cursor.GetArgument(j);
                    var prevParamName = prevParam.Spelling.ToString();
                    if (string.IsNullOrEmpty(prevParamName))
                        prevParamName = $"param{j}";
                    prevParamName = CleanParameterName(prevParamName, prevParam.Type);
                    prevParamName = EnsureUniqueParameterName(prevParamName, tempUsedNames);
                    tempUsedNames.Add(prevParamName);
                }
                paramName = EnsureUniqueParameterName(paramName, tempUsedNames);
                
                output.Add($"        /// <param name=\"{paramName}\">{GenerateParameterDocumentation(paramName, param.Type, cursor)}</param>");
            }
            
            output.Add($"        /// <returns>{GenerateReturnDocumentation(returnType, cursor)}</returns>");
            
            // Generate the function with EntryPoint and LIB_NAME constant
            var cleanFunctionName = CleanFunctionName(functionName);
            output.Add($"        [DllImport(LIB_NAME, EntryPoint = \"{functionName}\")]");
            output.Add($"        public static extern {returnType} {cleanFunctionName}({paramStr});");
            output.Add("");
            
            Console.WriteLine($"    Generated P/Invoke for '{functionName}': {returnType} {cleanFunctionName}({paramStr})");
        }
        
        private string CleanParameterName(string paramName, CXType paramType)
        {
            // Convert snake_case to lowerCamelCase and handle special cases
            return ConvertSnakeCaseToCamelCase(paramName);
        }
        
        private string EnsureUniqueParameterName(string paramName, HashSet<string> usedNames)
        {
            if (!usedNames.Contains(paramName))
                return paramName;
            
            // If the name is already used, append a number to make it unique
            int counter = 1;
            string uniqueName;
            do
            {
                uniqueName = $"{paramName}{counter}";
                counter++;
            } while (usedNames.Contains(uniqueName));
            
            return uniqueName;
        }
        
        private string CleanFunctionName(string functionName)
        {
            // Remove common prefixes to make function names cleaner
            if (functionName.StartsWith("RM_"))
                return functionName.Substring(3);
            return functionName;
        }
        
        private string CleanTypeName(string typeName)
        {
            // Remove common prefixes to make type names cleaner
            if (typeName.StartsWith("RM_"))
                typeName = typeName.Substring(3);

            // Special case checks for legacy compatibility.
            if (typeName == "PositionData")
                return "OpenDrivePositionData";

            return typeName;
        }
        
        private bool IsPrimitiveType(string typeSpelling)
        {
            return typeSpelling switch
            {
                "int" => true,
                "unsigned int" => true,
                "float" => true,
                "double" => true,
                "bool" => true,
                "char" => true,
                "unsigned char" => true,
                "short" => true,
                "unsigned short" => true,
                "long" => true,
                "unsigned long" => true,
                "long long" => true,
                "unsigned long long" => true,
                "size_t" => true,
                "ptrdiff_t" => true,
                "int8_t" => true,
                "uint8_t" => true,
                "int16_t" => true,
                "uint16_t" => true,
                "int32_t" => true,
                "uint32_t" => true,
                "int64_t" => true,
                "uint64_t" => true,
                "id_t" => true,
                _ => false
            };
        }
        
        private bool IsValidCSharpIdentifier(string name)
        {
            if (string.IsNullOrEmpty(name))
                return false;
                
            // Check if first character is valid (letter or underscore)
            if (!char.IsLetter(name[0]) && name[0] != '_')
                return false;
                
            // Check if remaining characters are valid (letters, digits, or underscores)
            for (int i = 1; i < name.Length; i++)
            {
                if (!char.IsLetterOrDigit(name[i]) && name[i] != '_')
                    return false;
            }
            
            return true;
        }

        private string ConvertSnakeCaseToCamelCase(string name)
        {
            if (string.IsNullOrEmpty(name))
                return name;

            // Handle special cases first
            switch (name)
            {
                case "road_lane_info":
                    return "laneInfo";
                case "relative_h":
                    return "relativeHeading";
            }

            // Check if the name contains underscores (snake_case)
            if (!name.Contains('_'))
                return name; // Not snake_case, return as-is

            // Convert snake_case to lowerCamelCase
            var parts = name.Split('_');
            if (parts.Length <= 1)
                return name; // Single word, return as-is

            var result = new System.Text.StringBuilder();
            
            // First part stays lowercase
            result.Append(parts[0].ToLowerInvariant());
            
            // Subsequent parts get first letter capitalized
            for (int i = 1; i < parts.Length; i++)
            {
                if (!string.IsNullOrEmpty(parts[i]))
                {
                    result.Append(char.ToUpperInvariant(parts[i][0]));
                    if (parts[i].Length > 1)
                        result.Append(parts[i].Substring(1).ToLowerInvariant());
                }
            }
            
            return result.ToString();
        }
        
        private string ApplySpecialCaseTransformations(string name)
        {
            // Apply special case transformations for struct member names
            switch (name)
            {
                case "townarterial":
                    return "townArterial";
                case "towncollector":
                    return "townCollector";
                case "townexpressway":
                    return "townExpressway";
                case "townlocal":
                    return "townLocal";
                case "townplaystreet":
                    return "townPlaystreet";
                case "townprivate":
                    return "townPrivate";
                // Handle capitalized versions as well
                case "Townarterial":
                    return "TownArterial";
                case "Towncollector":
                    return "TownCollector";
                case "Townexpressway":
                    return "TownExpressway";
                case "Townlocal":
                    return "TownLocal";
                case "Townplaystreet":
                    return "TownPlaystreet";
                case "Townprivate":
                    return "TownPrivate";
                default:
                    return name;
            }
        }
        
        private string GenerateFunctionDocumentation(string functionName, CXCursor cursor)
        {
            // First try to get documentation from the cursor's comment
            var rawComment = cursor.RawCommentText;
            var commentText = rawComment.ToString();
            if (!string.IsNullOrWhiteSpace(commentText))
            {
                // Clean up the comment text - remove comment markers and extra whitespace
                return CleanCommentText(commentText);
            }
            
            // If no comment found, try to get parsed comment
            var parsedComment = cursor.ParsedComment;
            var briefText = ExtractBriefFromParsedComment(parsedComment);
            if (!string.IsNullOrWhiteSpace(briefText))
            {
                return briefText;
            }
            
            // Fallback to pattern-based documentation if no comments found
            return functionName switch
            {
                "RM_SetLanePosition" => "Set position from road coordinates, world coordinates being calculated",
                "RM_Init" => "Initialize the Road Manager",
                "RM_Close" => "Close the Road Manager and clean up resources",
                "RM_CreatePosition" => "Create a new position object",
                "RM_DeletePosition" => "Delete a position object",
                "RM_GetPositionData" => "Get position data for the specified position object",
                _ => $"Call {functionName}"
            };
        }
        
        private string CleanCommentText(string rawComment)
        {
            if (string.IsNullOrWhiteSpace(rawComment))
                return string.Empty;
                
            var lines = rawComment.Split('\n');
            var cleanedLines = new List<string>();
            
            foreach (var line in lines)
            {
                var trimmedLine = line.Trim();
                
                // Remove common comment markers
                if (trimmedLine.StartsWith("///"))
                    trimmedLine = trimmedLine.Substring(3).Trim();
                else if (trimmedLine.StartsWith("//"))
                    trimmedLine = trimmedLine.Substring(2).Trim();
                else if (trimmedLine.StartsWith("/*"))
                    trimmedLine = trimmedLine.Substring(2).Trim();
                else if (trimmedLine.StartsWith("*"))
                    trimmedLine = trimmedLine.Substring(1).Trim();
                else if (trimmedLine.EndsWith("*/"))
                    trimmedLine = trimmedLine.Substring(0, trimmedLine.Length - 2).Trim();
                
                // Skip empty lines and common Doxygen commands
                if (!string.IsNullOrWhiteSpace(trimmedLine) && 
                    !trimmedLine.StartsWith("@") && 
                    !trimmedLine.StartsWith("\\"))
                {
                    cleanedLines.Add(trimmedLine);
                }
            }
            
            // Join the lines and return the first meaningful sentence
            var result = string.Join(" ", cleanedLines).Trim();
            if (result.Length > 200) // Truncate if too long
            {
                var firstSentence = result.IndexOf('.');
                if (firstSentence > 0 && firstSentence < 200)
                    result = result.Substring(0, firstSentence + 1);
                else
                    result = result.Substring(0, 200) + "...";
            }
            
            return result;
        }
        
        private string ExtractBriefFromParsedComment(CXComment parsedComment)
        {
            // This is a simplified extraction - ClangSharp's comment API can be complex
            // For now, we'll try to get the basic text content
            try
            {
                var kind = parsedComment.Kind;
                if (kind == CXCommentKind.CXComment_FullComment)
                {
                    // Try to extract brief description from full comment
                    var numChildren = parsedComment.NumChildren;
                    for (uint i = 0; i < numChildren; i++)
                    {
                        var child = parsedComment.GetChild(i);
                        if (child.Kind == CXCommentKind.CXComment_Paragraph)
                        {
                            // Extract text from paragraph
                            var text = ExtractTextFromComment(child);
                            if (!string.IsNullOrWhiteSpace(text))
                                return text.Trim();
                        }
                    }
                }
                else if (kind == CXCommentKind.CXComment_Paragraph)
                {
                    return ExtractTextFromComment(parsedComment).Trim();
                }
            }
            catch
            {
                // If comment parsing fails, return empty string
            }
            
            return string.Empty;
        }
        
        private string ExtractTextFromComment(CXComment comment)
        {
            var result = new List<string>();
            
            try
            {
                var numChildren = comment.NumChildren;
                for (uint i = 0; i < numChildren; i++)
                {
                    var child = comment.GetChild(i);
                    if (child.Kind == CXCommentKind.CXComment_Text)
                    {
                        // Use a simpler approach to get text from comment
                        var text = child.ToString();
                        if (!string.IsNullOrWhiteSpace(text))
                            result.Add(text.Trim());
                    }
                }
            }
            catch
            {
                // If extraction fails, return empty
            }
            
            return string.Join(" ", result);
        }
        
        private string GenerateParameterDocumentation(string paramName, CXType paramType, CXCursor functionCursor)
        {
            // Try to extract parameter documentation from function comments
            var paramDoc = ExtractParameterDocFromComment(functionCursor, paramName);
            if (!string.IsNullOrWhiteSpace(paramDoc))
                return paramDoc;
            
            // Fallback to pattern-based documentation
            return paramName switch
            {
                "index" => "Handle to the position object",
                "roadId" => "The OpenDRIVE road ID",
                "laneId" => "Lane specifier",
                "laneOffset" => "Offset from lane center",
                "s" => "Distance along the specified road",
                "align" => "If true the heading will be reset to the lane driving direction (typically only at initialization)",
                _ => $"Parameter {paramName}"
            };
        }
        
        private string ExtractParameterDocFromComment(CXCursor functionCursor, string paramName)
        {
            // Try to extract parameter documentation from raw comment
            var rawComment = functionCursor.RawCommentText;
            var commentText = rawComment.ToString();
            if (!string.IsNullOrWhiteSpace(commentText))
            {
                // Look for @param or \param documentation
                var lines = commentText.Split('\n');
                foreach (var line in lines)
                {
                    var trimmedLine = line.Trim();
                    
                    // Remove comment markers
                    if (trimmedLine.StartsWith("///"))
                        trimmedLine = trimmedLine.Substring(3).Trim();
                    else if (trimmedLine.StartsWith("//"))
                        trimmedLine = trimmedLine.Substring(2).Trim();
                    else if (trimmedLine.StartsWith("*"))
                        trimmedLine = trimmedLine.Substring(1).Trim();
                    
                    // Look for parameter documentation patterns
                    if (trimmedLine.StartsWith($"@param {paramName}") || 
                        trimmedLine.StartsWith($"\\param {paramName}"))
                    {
                        var docStart = trimmedLine.IndexOf(paramName) + paramName.Length;
                        if (docStart < trimmedLine.Length)
                        {
                            return trimmedLine.Substring(docStart).Trim();
                        }
                    }
                    
                    // Also look for inline parameter documentation like "paramName - description"
                    if (trimmedLine.Contains($"{paramName} -") || trimmedLine.Contains($"{paramName}:"))
                    {
                        var separator = trimmedLine.Contains($"{paramName} -") ? " -" : ":";
                        var parts = trimmedLine.Split(new[] { separator }, StringSplitOptions.RemoveEmptyEntries);
                        if (parts.Length > 1 && parts[0].Trim().EndsWith(paramName))
                        {
                            return parts[1].Trim();
                        }
                    }
                }
            }
            
            return string.Empty;
        }
        
        private string GenerateReturnDocumentation(string returnType, CXCursor functionCursor)
        {
            // Try to extract return documentation from function comments
            var returnDoc = ExtractReturnDocFromComment(functionCursor);
            if (!string.IsNullOrWhiteSpace(returnDoc))
                return returnDoc;
                
            // Fallback to pattern-based documentation
            return returnType switch
            {
                "int" => "0 if successful, -1 if not",
                "void" => "No return value",
                "bool" => "True if successful, false otherwise",
                _ => $"Returns {returnType}"
            };
        }
        
        private string ExtractReturnDocFromComment(CXCursor functionCursor)
        {
            // Try to extract return documentation from raw comment
            var rawComment = functionCursor.RawCommentText;
            var commentText = rawComment.ToString();
            if (!string.IsNullOrWhiteSpace(commentText))
            {
                // Look for @return, @returns, \return, or \returns documentation
                var lines = commentText.Split('\n');
                foreach (var line in lines)
                {
                    var trimmedLine = line.Trim();
                    
                    // Remove comment markers
                    if (trimmedLine.StartsWith("///"))
                        trimmedLine = trimmedLine.Substring(3).Trim();
                    else if (trimmedLine.StartsWith("//"))
                        trimmedLine = trimmedLine.Substring(2).Trim();
                    else if (trimmedLine.StartsWith("*"))
                        trimmedLine = trimmedLine.Substring(1).Trim();
                    
                    // Look for return documentation patterns
                    if (trimmedLine.StartsWith("@return") || 
                        trimmedLine.StartsWith("@returns") ||
                        trimmedLine.StartsWith("\\return") ||
                        trimmedLine.StartsWith("\\returns"))
                    {
                        var returnKeyword = trimmedLine.Split(' ')[0];
                        var docStart = trimmedLine.IndexOf(returnKeyword) + returnKeyword.Length;
                        if (docStart < trimmedLine.Length)
                        {
                            return trimmedLine.Substring(docStart).Trim();
                        }
                    }
                }
            }
            
            return string.Empty;
        }

        private string ExtractFieldComment(CXCursor fieldCursor)
        {
            // Try to extract comment from field cursor
            var rawComment = fieldCursor.RawCommentText;
            var commentText = rawComment.ToString();
            if (!string.IsNullOrWhiteSpace(commentText))
            {
                return CleanCommentText(commentText);
            }

            // If no direct comment, try to get it from parsed comment
            var parsedComment = fieldCursor.ParsedComment;
            var briefText = ExtractBriefFromParsedComment(parsedComment);
            if (!string.IsNullOrWhiteSpace(briefText))
            {
                return briefText;
            }

            return string.Empty;
        }

        private string FindCommonPrefix(List<string> literals, int minLiterals = 3)
        {
            if (literals.Count < minLiterals)
                return string.Empty;

            // Remove RM_ prefix from all literals first for analysis
            var cleanedLiterals = literals.Select(lit => lit.StartsWith("RM_") ? lit.Substring(3) : lit).ToList();

            if (cleanedLiterals.Count == 0)
                return string.Empty;

            // Find the shortest literal to limit our search
            int minLength = cleanedLiterals.Min(lit => lit.Length);
            if (minLength == 0)
                return string.Empty;

            // Find the longest common prefix ending with underscore
            string commonPrefix = string.Empty;
            
            for (int i = 1; i <= minLength; i++)
            {
                string potentialPrefix = cleanedLiterals[0].Substring(0, i);
                
                // Check if all literals start with this prefix
                if (cleanedLiterals.All(lit => lit.StartsWith(potentialPrefix, StringComparison.OrdinalIgnoreCase)))
                {
                    // If the prefix ends with underscore, it's a good candidate
                    if (potentialPrefix.EndsWith("_"))
                    {
                        commonPrefix = potentialPrefix;
                    }
                    // If we're at the end and no underscore, check if all literals are identical up to here
                    else if (i == minLength)
                    {
                        // Look for the last underscore in the potential prefix
                        int lastUnderscore = potentialPrefix.LastIndexOf('_');
                        if (lastUnderscore > 0)
                        {
                            commonPrefix = potentialPrefix.Substring(0, lastUnderscore + 1);
                        }
                    }
                }
                else
                {
                    break; // No longer a common prefix
                }
            }

            // If we didn't find a prefix ending with underscore, look for longest prefix followed by underscore
            if (string.IsNullOrEmpty(commonPrefix) && cleanedLiterals.Count > 0)
            {
                // Try to find common prefix by checking progressively
                string firstLiteral = cleanedLiterals[0];
                for (int i = firstLiteral.Length; i > 0; i--)
                {
                    if (firstLiteral[i - 1] == '_')
                    {
                        string candidate = firstLiteral.Substring(0, i);
                        if (cleanedLiterals.All(lit => lit.StartsWith(candidate, StringComparison.OrdinalIgnoreCase)))
                        {
                            commonPrefix = candidate;
                            break;
                        }
                    }
                }
            }

            return commonPrefix;
        }

        private string TransformEnumLiteralWithCommonPrefix(string enumName, string literalName, string commonPrefix)
        {
            // Remove RM_ prefix if present (legacy handling)
            if (literalName.StartsWith("RM_"))
                literalName = literalName.Substring(3);

            // First try to remove common prefix if it exists and is meaningful
            if (!string.IsNullOrEmpty(commonPrefix) && commonPrefix.Length > 1)
            {
                if (literalName.StartsWith(commonPrefix, StringComparison.OrdinalIgnoreCase))
                {
                    literalName = literalName.Substring(commonPrefix.Length);
                }
            }
            // Otherwise, fall back to enum name prefix removal
            else
            {
                // Check if the literal starts with the enum name (case-insensitive)
                if (literalName.StartsWith(enumName + "_", StringComparison.OrdinalIgnoreCase))
                {
                    // Remove the enum name and the following underscore
                    literalName = literalName.Substring(enumName.Length + 1);
                }
                else if (literalName.StartsWith(enumName, StringComparison.OrdinalIgnoreCase))
                {
                    // Remove just the enum name if no underscore follows
                    literalName = literalName.Substring(enumName.Length);
                    // Remove leading underscore if it exists after enum name removal
                    if (literalName.StartsWith("_"))
                        literalName = literalName.Substring(1);
                }
            }

            // Convert from UPPER_SNAKE_CASE to UpperCamelCase
            var result = ConvertToUpperCamelCase(literalName);
            
            // Apply special case transformations
            return ApplySpecialCaseTransformations(result);
        }

        private string TransformEnumLiteral(string enumName, string literalName)
        {
            // Remove RM_ prefix if present (legacy handling)
            if (literalName.StartsWith("RM_"))
                literalName = literalName.Substring(3);

            // Check if the literal starts with the enum name (case-insensitive)
            if (literalName.StartsWith(enumName + "_", StringComparison.OrdinalIgnoreCase))
            {
                // Remove the enum name and the following underscore
                literalName = literalName.Substring(enumName.Length + 1);
            }
            else if (literalName.StartsWith(enumName, StringComparison.OrdinalIgnoreCase))
            {
                // Remove just the enum name if no underscore follows
                literalName = literalName.Substring(enumName.Length);
                // Remove leading underscore if it exists after enum name removal
                if (literalName.StartsWith("_"))
                    literalName = literalName.Substring(1);
            }

            // Convert from UPPER_SNAKE_CASE to UpperCamelCase
            return ConvertToUpperCamelCase(literalName);
        }

        private string ConvertToUpperCamelCase(string input)
        {
            if (string.IsNullOrEmpty(input))
                return input;

            // Split by underscores and convert each part
            var parts = input.Split('_', StringSplitOptions.RemoveEmptyEntries);
            var result = new System.Text.StringBuilder();

            foreach (var part in parts)
            {
                if (!string.IsNullOrEmpty(part))
                {
                    // Capitalize first letter and make the rest lowercase
                    result.Append(char.ToUpper(part[0]));
                    if (part.Length > 1)
                        result.Append(part.Substring(1).ToLower());
                }
            }

            return result.ToString();
        }

        private void ProcessEnum(CXCursor cursor, string enumName)
        {
            if (enums.Contains(enumName))
            {
                Console.WriteLine($"    Enum '{enumName}' already processed, skipping");
                return;
            }

            enums.Add(enumName);
            
            var cleanEnumName = CleanTypeName(enumName);
            
            // First, collect all enum literals
            var enumLiterals = new List<(string name, long value)>();
            
            cursor.VisitChildren((child, parent, clientData) =>
            {
                if (child.Kind == CXCursorKind.CXCursor_EnumConstantDecl)
                {
                    var constantName = child.Spelling.ToString();
                    var constantValue = child.EnumConstantDeclValue;
                    enumLiterals.Add((constantName, constantValue));
                }
                return CXChildVisitResult.CXChildVisit_Continue;
            }, default(CXClientData));

            // Analyze for common prefix
            var literalNames = enumLiterals.Select(lit => lit.name).ToList();
            var commonPrefix = FindCommonPrefix(literalNames);
            
            Console.WriteLine($"    Processing enum '{enumName}' with {enumLiterals.Count} literals");
            if (!string.IsNullOrEmpty(commonPrefix))
            {
                Console.WriteLine($"    Detected common prefix: '{commonPrefix}'");
            }
            
            // Add comment indicating source file if it's from a dependency
            if (referencedTypes.Contains(enumName))
            {
                var location = cursor.Location;
                location.GetFileLocation(out var filename, out _, out _, out _);
                var filenameStr = filename.ToString();
                if (!string.IsNullOrEmpty(filenameStr))
                {
                    var shortFilename = Path.GetFileName(filenameStr);
                    output.Add($"    // Enum from dependency: {shortFilename}");
                }
            }

            output.Add($"    public enum {cleanEnumName}");
            output.Add("    {");

            // Transform each literal using the detected common prefix
            foreach (var (name, value) in enumLiterals)
            {
                var transformedName = TransformEnumLiteralWithCommonPrefix(enumName, name, commonPrefix);
                output.Add($"        {transformedName} = {value},");
            }

            output.Add("    }");
            output.Add("");
        }

        private string MapType(CXType type, TypeMappingContext context = TypeMappingContext.FunctionParameter) => MapType(type, context, out _, out _, out _);

        private string MapType(CXType type, TypeMappingContext context, out bool isByRef, out bool isReadOnly, out bool isOutParam)
        {
            isByRef = false;
            isReadOnly = false;
            isOutParam = false;
            string typeSpelling = type.Spelling.ToString();

            // Pointer handling
            if (type.kind == CXTypeKind.CXType_Pointer)
            {
                var pointee = type.PointeeType;
                
                // Handle both CXType_Record and CXType_Elaborated (which is used for struct types)
                if (pointee.kind == CXTypeKind.CXType_Record || pointee.kind == CXTypeKind.CXType_Elaborated)
                {
                    var pointeeTypeName = pointee.Spelling.ToString();
                    var cleanedPointeeTypeName = CleanTypeName(pointeeTypeName);
                    
                    // Check if this is a type we're wrapping (either from main file or dependencies)
                    // Check both the original name and the cleaned name
                    if (IsMainFileDeclaration(pointeeTypeName) || referencedTypes.Contains(pointeeTypeName) ||
                        IsMainFileDeclaration(cleanedPointeeTypeName) || referencedTypes.Contains(cleanedPointeeTypeName) ||
                        structs.ContainsKey(pointeeTypeName) || structs.ContainsKey(cleanedPointeeTypeName))
                    {
                        isByRef = true;
                        isReadOnly = type.IsConstQualified;
                        return CleanTypeName(pointeeTypeName);
                    }
                    // Check if this looks like a named struct type that should be treated as ref
                    // This handles cases where struct types weren't properly discovered in dependency processing
                    else if (!string.IsNullOrEmpty(pointeeTypeName) && 
                             !pointeeTypeName.StartsWith("_") && // Avoid internal/anonymous structs
                             !pointeeTypeName.Contains("::") && // Avoid complex nested types
                             pointeeTypeName.Length > 1) // Avoid single-character types
                    {
                        // Treat as ref parameter for named struct types
                        isByRef = true;
                        isReadOnly = type.IsConstQualified;
                        return CleanTypeName(pointeeTypeName);
                    }
                    else
                    {
                        // Unknown struct type, treat as IntPtr
                        return "IntPtr";
                    }
                }
                var pointeeSpelling = pointee.Spelling.ToString();
                if ((pointeeSpelling == "char" || pointeeSpelling == "const char") && (type.IsConstQualified || pointee.IsConstQualified))
                {
                    // For function parameters, map const char* to string
                    // For struct fields, keep as IntPtr to avoid marshaling issues
                    if (context == TypeMappingContext.FunctionParameter)
                        return "string";
                    else
                        return "IntPtr";
                }
                
                // Check if this is a primitive type pointer
                if (IsPrimitiveType(pointeeSpelling) && context == TypeMappingContext.FunctionParameter)
                {
                    // For primitive type pointers in function parameters, use out parameter
                    isByRef = true;
                    isReadOnly = false; // out parameters are never read-only
                    isOutParam = true; // This should be an out parameter
                    
                    // Map the primitive type to its C# equivalent
                    return pointeeSpelling switch
                    {
                        "int" => "int",
                        "unsigned int" => "uint",
                        "float" => "float",
                        "double" => "double",
                        "bool" => "bool",
                        "char" => "byte",
                        "unsigned char" => "byte",
                        "short" => "short",
                        "unsigned short" => "ushort",
                        "long" => "long",
                        "unsigned long" => "ulong",
                        "long long" => "long",
                        "unsigned long long" => "ulong",
                        "size_t" => "nuint",
                        "ptrdiff_t" => "nint",
                        "int8_t" => "sbyte",
                        "uint8_t" => "byte",
                        "int16_t" => "short",
                        "uint16_t" => "ushort",
                        "int32_t" => "int",
                        "uint32_t" => "uint",
                        "int64_t" => "long",
                        "uint64_t" => "ulong",
                        "id_t" => "int",
                        _ => "IntPtr" // Fallback for unknown types
                    };
                }
                
                return "IntPtr";
            }
            
            // Handle record types (structs) directly
            if (type.kind == CXTypeKind.CXType_Record || type.kind == CXTypeKind.CXType_Elaborated)
            {
                var recordTypeName = type.Spelling.ToString();
                var cleanedRecordTypeName = CleanTypeName(recordTypeName);
                
                // Check if this is a type we're wrapping (main file, dependencies, or already processed structs)
                if (IsMainFileDeclaration(recordTypeName) || referencedTypes.Contains(recordTypeName) ||
                    IsMainFileDeclaration(cleanedRecordTypeName) || referencedTypes.Contains(cleanedRecordTypeName) ||
                    structs.ContainsKey(recordTypeName) || structs.ContainsKey(cleanedRecordTypeName))
                {
                    return CleanTypeName(recordTypeName);
                }
            }
            
            // Handle enum types
            if (type.kind == CXTypeKind.CXType_Enum)
            {
                var enumTypeName = type.Spelling.ToString();
                if (IsMainFileDeclaration(enumTypeName) || referencedTypes.Contains(enumTypeName))
                {
                    return CleanTypeName(enumTypeName);
                }
                // For unknown enums, default to int
                return "int";
            }

            return typeSpelling switch
            {
                "int" => "int",
                "unsigned int" => "uint",
                "float" => "float",
                "double" => "double",
                "bool" => "bool",
                "char" => "byte",
                "unsigned char" => "byte",
                "short" => "short",
                "unsigned short" => "ushort",
                "long" => "long",
                "unsigned long" => "ulong",
                "long long" => "long",
                "unsigned long long" => "ulong",
                "size_t" => "nuint",
                "ptrdiff_t" => "nint",
                "int8_t" => "sbyte",
                "uint8_t" => "byte",
                "int16_t" => "short",
                "uint16_t" => "ushort",
                "int32_t" => "int",
                "uint32_t" => "uint",
                "int64_t" => "long",
                "uint64_t" => "ulong",
                "id_t" => "int",  // Map id_t to int for OpenDRIVE road/lane IDs
                "void" => "void",
                _ => "IntPtr"
            };
        }

        // Method to analyze comments for roadmanager:: references and discover additional enums
        private HashSet<string> AnalyzeCommentsForRoadManagerTypes(CXCursor rootCursor)
        {
            var discoveredTypes = new HashSet<string>();
            Console.WriteLine("Analyzing comments for roadmanager:: references...");
            
            VisitCursor(rootCursor, (cursor) =>
            {
                var rawComment = cursor.RawCommentText.ToString();
                if (!string.IsNullOrWhiteSpace(rawComment))
                {
                    var roadmanagerReferences = ExtractRoadManagerReferences(rawComment);
                    foreach (var reference in roadmanagerReferences)
                    {
                        discoveredTypes.Add(reference);
                        Console.WriteLine($"  Found roadmanager reference: {reference}");
                    }
                }
                return true; // Continue visiting
            });
            
            return discoveredTypes;
        }

        // Extract roadmanager:: references from comment text
        private List<string> ExtractRoadManagerReferences(string commentText)
        {
            var references = new List<string>();
            var lines = commentText.Split('\n');
            
            foreach (var line in lines)
            {
                var cleanLine = line.Trim();
                // Look for roadmanager:: patterns
                var roadmanagerIndex = cleanLine.IndexOf("roadmanager::");
                if (roadmanagerIndex >= 0)
                {
                    // Extract the full type reference
                    var startIndex = roadmanagerIndex;
                    var endIndex = startIndex;
                    
                    // Find the end of the type reference (until space, comma, or end of line)
                    while (endIndex < cleanLine.Length && 
                           !char.IsWhiteSpace(cleanLine[endIndex]) && 
                           cleanLine[endIndex] != ',' && 
                           cleanLine[endIndex] != '.' && 
                           cleanLine[endIndex] != ')' &&
                           cleanLine[endIndex] != ']')
                    {
                        endIndex++;
                    }
                    
                    var reference = cleanLine.Substring(startIndex, endIndex - startIndex);
                    if (!string.IsNullOrWhiteSpace(reference) && !references.Contains(reference))
                    {
                        references.Add(reference);
                    }
                }
            }
            
            return references;
        }

        // Find enum definitions for discovered roadmanager types
        private void DiscoverEnumsFromRoadManagerReferences(HashSet<string> roadmanagerReferences, CXCursor rootCursor)
        {
            Console.WriteLine("Searching for enum definitions from roadmanager references...");
            Console.WriteLine($"Total roadmanager references found: {roadmanagerReferences.Count}");
            
            if (roadmanagerReferences.Count == 0)
            {
                Console.WriteLine("No roadmanager references found in comments.");
                return;
            }
            
            // First, try to find enums in the current translation unit
            var currentEnums = FindAllEnums(rootCursor);
            Console.WriteLine($"Found {currentEnums.Count} enums in current translation unit");
            
            // Track which enums we need to find
            var missingEnums = new HashSet<string>();
            
            foreach (var reference in roadmanagerReferences)
            {
                var parts = reference.Split(new[] { "::" }, StringSplitOptions.RemoveEmptyEntries);
                if (parts.Length > 0)
                {
                    var enumName = parts[parts.Length - 1];
                    Console.WriteLine($"  Looking for enum: {enumName} from reference: {reference}");
                    
                    if (currentEnums.ContainsKey(enumName))
                    {
                        Console.WriteLine($"  Found {enumName} in current translation unit!");
                        referencedTypes.Add(enumName);
                        typeDefinitions[enumName] = currentEnums[enumName];
                    }
                    else
                    {
                        Console.WriteLine($"  Enum {enumName} not found in current translation unit");
                        missingEnums.Add(enumName);
                    }
                }
            }
            
            // If we have missing enums, try to find them in related header files
            if (missingEnums.Count > 0)
            {
                Console.WriteLine($"Attempting to find {missingEnums.Count} missing enums in related header files...");
                SearchAdditionalHeaderFiles(missingEnums);
            }
        }

        // Search for enums in additional header files
        private void SearchAdditionalHeaderFiles(HashSet<string> missingEnums)
        {
            // List of potential header files that might contain roadmanager enums
            var potentialHeaders = new List<string>
            {
                "RoadManager.hpp",
                "../RoadManager.hpp",
                "RoadManager.h",
                "../RoadManager.h"
            };
            
            foreach (var headerPath in potentialHeaders)
            {
                try
                {
                    var fullPath = Path.GetFullPath(headerPath);
                    if (File.Exists(fullPath))
                    {
                        Console.WriteLine($"  Parsing additional header: {fullPath}");
                        var foundEnums = ParseHeaderForEnums(fullPath, missingEnums);
                        
                        foreach (var enumPair in foundEnums)
                        {
                            Console.WriteLine($"  Found missing enum: {enumPair.Key}");
                            referencedTypes.Add(enumPair.Key);
                            typeDefinitions[enumPair.Key] = enumPair.Value;
                            missingEnums.Remove(enumPair.Key);
                        }
                        
                        // If we found all missing enums, we can stop
                        if (missingEnums.Count == 0)
                        {
                            Console.WriteLine("  All missing enums found!");
                            break;
                        }
                    }
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"  Error parsing {headerPath}: {ex.Message}");
                }
            }
            
            if (missingEnums.Count > 0)
            {
                Console.WriteLine($"  Still missing {missingEnums.Count} enums: {string.Join(", ", missingEnums)}");
            }
        }

        // Parse a header file specifically looking for certain enums
        private Dictionary<string, CXCursor> ParseHeaderForEnums(string headerPath, HashSet<string> targetEnums)
        {
            var foundEnums = new Dictionary<string, CXCursor>();
            
            var index = CXIndex.Create();
            var compilerArgs = new string[]
            {
                "-std=c++17",
                "-DWIN32",
                "-D_WIN32",
                "-I.",
                "-I..",
                "-fparse-all-comments",
                "-Wno-pragma-once-outside-header"
            };
            
            var translationUnit = CXTranslationUnit.Parse(index, headerPath, compilerArgs, 
                null, CXTranslationUnit_Flags.CXTranslationUnit_None);
            
            if (translationUnit.Handle != IntPtr.Zero)
            {
                var cursor = translationUnit.Cursor;
                var allEnums = FindAllEnums(cursor);
                
                foreach (var enumName in targetEnums)
                {
                    if (allEnums.ContainsKey(enumName))
                    {
                        foundEnums[enumName] = allEnums[enumName];
                    }
                }
                
                // Keep the translation unit alive so cursors remain valid
                additionalTranslationUnits.Add(translationUnit);
            }
            
            index.Dispose();
            return foundEnums;
        }

        // Find all enums in the translation unit
        private Dictionary<string, CXCursor> FindAllEnums(CXCursor cursor)
        {
            var enums = new Dictionary<string, CXCursor>();
            
            VisitCursor(cursor, (child) =>
            {
                if (child.Kind == CXCursorKind.CXCursor_EnumDecl)
                {
                    var enumName = child.Spelling.ToString();
                    if (!string.IsNullOrEmpty(enumName) && !enums.ContainsKey(enumName))
                    {
                        enums[enumName] = child;
                    }
                }
                return true;
            });
            
            return enums;
        }

        // Recursively search for enum definition by name
        private CXCursor FindEnumDefinition(CXCursor cursor, string enumName)
        {
            CXCursor foundCursor = default;
            
            VisitCursor(cursor, (child) =>
            {
                if (child.Kind == CXCursorKind.CXCursor_EnumDecl)
                {
                    var childEnumName = child.Spelling.ToString();
                    if (childEnumName == enumName)
                    {
                        foundCursor = child;
                        return false; // Found it, stop searching
                    }
                }
                return true; // Continue searching
            });
            
            return foundCursor;
        }

        // Helper method to visit cursors recursively
        private void VisitCursor(CXCursor cursor, Func<CXCursor, bool> visitor)
        {
            cursor.VisitChildren((child, parent, clientData) =>
            {
                if (!visitor(child))
                    return CXChildVisitResult.CXChildVisit_Break;
                
                VisitCursor(child, visitor);
                return CXChildVisitResult.CXChildVisit_Continue;
            }, clientData: default);
        }
    }

    public class StructInfo
    {
        public string Name { get; set; } = string.Empty;
        public List<FieldInfo> Fields { get; set; } = new List<FieldInfo>();
    }

    public class FieldInfo
    {
        public string Name { get; set; } = string.Empty;
        public string Type { get; set; } = string.Empty;
        public string Comment { get; set; } = string.Empty;
    }
}