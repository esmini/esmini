#!/usr/bin/env python3
"""
C# Wrapper Generator for esminiLib.hpp

This script parses a C/C++ header file and generates a corresponding C#
P/Invoke wrapper. It is designed to handle macros, enums, structs, and
function declarations, including complex types like function pointers.

Created with assistance of Gemini 2.5 and 3.0 preview, based on following initial prompt:

Create a Python script that generates a C# wrapper for the C++ headerfile esminiLib.hpp, with the following features:
- Default namespace = ESMini
- Default class name = ESMiniLib
- Keep prefix "SE_" for all elements, including structs
- define symbol WIN32 and obey #ifdef macros
- Include all macros (#define) since they define good to know constants
- Include all enums
- Include all structs
- Reuse all comments (documentation), including detailed info on function arguments and return value
- Replace all C++ types with C# corresponding ones, with following explicit requirements:
- Handle function pointers in arguments, for example "void (*fnPtr)(SE_Image *, void *)", creating delegates with name mapping to the Register*Callback functions
- Return values of type "const char *" should be translated into IntPtr, make sure to handle the case where the "*" is attached to the function name, like this: "const char *MyFunc"
- Function arguments including a single "*" (pointer) should be translated into "out"
- Function arguments including "**" (pointer to a char* for example) shall be translated into out IntPtr
- Function arguments "void*" and "void *" shall be replaced with IntPtr
- Function arguments prefix "const" replaced by "in"

"""

import argparse
import re
from datetime import datetime
from dataclasses import dataclass, field
from typing import List, Optional, Tuple, Dict, Set

# --- Data-driven C# Type Mapping ---

# Maps fundamental C types to their C# equivalents.
BASE_TYPE_MAP = {
    "void": "void",
    "bool": "bool",
    "char": "byte",
    "signed char": "sbyte",
    "unsigned char": "byte",
    "short": "short",
    "unsigned short": "ushort",
    "int": "int",
    "unsigned int": "uint",
    "long": "int",  # Win32 assumption: long is 32-bit
    "unsigned long": "uint",
    "long long": "long",
    "unsigned long long": "ulong",
    "float": "float",
    "double": "double",
    "size_t": "UIntPtr",
    "uint32_t": "uint",
    "int32_t": "int",
    "uint64_t": "ulong",
    "int64_t": "long",
    "id_t": "uint",
}

# --- Abstract Syntax Tree (AST) Representation ---

@dataclass
class MacroDef:
    """Represents a #define macro."""
    name: str
    value: str
    comments: List[str] = field(default_factory=list)

@dataclass
class EnumItem:
    """Represents a single item within an enum."""
    name: str
    value: Optional[str]
    comment: str = ""

@dataclass
class EnumDef:
    """Represents a typedef enum."""
    name: str
    items: List[EnumItem]
    comments: List[str] = field(default_factory=list)

@dataclass
class StructField:
    """Represents a field within a struct."""
    ctype: str
    name: str
    is_const: bool
    ptr_level: int
    array_len: Optional[int] = None
    comment: str = ""

@dataclass
class StructDef:
    """Represents a typedef struct."""
    name: str
    fields: List[StructField]
    comments: List[str] = field(default_factory=list)

@dataclass
class ArgDef:
    """Represents a simple function argument."""
    raw: str
    ctype: str
    name: str
    is_const: bool
    ptr_level: int

@dataclass
class FuncPtrArg:
    """Represents a function pointer argument."""
    raw: str
    name: str
    ret_ctype: str
    ret_is_const: bool
    ret_ptr_level: int
    args: List[ArgDef]

@dataclass
class FunctionDef:
    """Represents a function declaration."""
    name: str
    ret_ctype: str
    ret_is_const: bool
    ret_ptr_level: int
    args: List[object]  # Can contain ArgDef or FuncPtrArg
    comments: List[str] = field(default_factory=list)

# --- Parsing Logic ---

def normalize_ws(s: str) -> str:
    """Replaces all whitespace sequences with a single space."""
    return re.sub(r"\s+", " ", s.strip())

def strip_comment_markers(line: str) -> str:
    """Removes C/C++ comment markers from a line."""
    s = line.strip()
    s = re.sub(r"^\s*/\*\*?", "", s)
    s = re.sub(r"\*/\s*$", "", s)
    s = re.sub(r"^\s*\*\s?", "", s)
    s = re.sub(r"^\s*//\s?", "", s)
    return s.strip()

def to_xml_doc_lines(comments: List[str], indent: str = "") -> List[str]:
    """Converts a list of C comment lines to C# XML documentation."""
    if not comments:
        return []

    summary_lines = []
    params = {}  # Use a dict to store param descriptions: {name: [lines]}
    return_lines = []

    # State machine: 'summary', 'param', 'return'
    current_section = 'summary'
    current_param_name = None

    for c in comments:
        text = strip_comment_markers(c)

        # Doxygen tags can start a new section.
        # Handles @ or \ and optional [in], [out].
        brief_match = re.match(r"[@\\]brief\s+(.*)", text)
        # Groups: 1=[in/out], 2=name, 3=description
        param_match = re.match(r"[@\\]param(?:\[(in|out|in,out)\])?\s+(\w+)\s*(.*)", text)
        return_match = re.match(r"[@\\]return\s*(.*)", text)

        if brief_match:
            current_section = 'summary'
            current_param_name = None
            line_content = brief_match.group(1).strip()
            if line_content:
                summary_lines.append(line_content)
        elif param_match:
            current_section = 'param'
            current_param_name = param_match.group(2)
            line_content = param_match.group(3).strip()
            if current_param_name not in params:
                params[current_param_name] = []
            if line_content:
                params[current_param_name].append(line_content)
        elif return_match:
            current_section = 'return'
            current_param_name = None
            line_content = return_match.group(1).strip()
            if line_content:
                return_lines.append(line_content)
        else:
            # This is a continuation of the previous section
            line_content = text.strip()
            if not line_content:
                # An empty line might separate sections. Let's treat it as a separator
                # and not append it, but it doesn't change the current section.
                continue

            if current_section == 'summary':
                summary_lines.append(line_content)
            elif current_section == 'param' and current_param_name:
                params[current_param_name].append(line_content)
            elif current_section == 'return':
                return_lines.append(line_content)

    # --- Assemble the XML block ---
    xml_lines = []

    # Helper to escape XML special characters
    def escape(s):
        return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")

    if summary_lines:
        xml_lines.append(f"{indent}/// <summary>")
        for line in summary_lines:
            xml_lines.append(f"{indent}/// {escape(line)}")
        xml_lines.append(f"{indent}/// </summary>")

    for name, desc_lines in params.items():
        # Join the description lines for a single <param> tag
        full_desc = escape(" ".join(desc_lines))
        xml_lines.append(f'{indent}/// <param name="{name}">{full_desc}</param>')

    if return_lines:
        # Join the description lines for a single <returns> tag
        full_desc = escape(" ".join(return_lines))
        xml_lines.append(f'{indent}/// <returns>{full_desc}</returns>')

    return xml_lines

def split_args(arg_blob: str) -> List[str]:
    """Splits a C argument string, respecting parentheses for function pointers."""
    arg_blob = arg_blob.strip()
    if not arg_blob or arg_blob == "void":
        return []
    res = []
    cur = []
    depth = 0
    for ch in arg_blob:
        if ch == "," and depth == 0:
            part = "".join(cur).strip()
            if part:
                res.append(part)
            cur = []
            continue
        if ch == "(":
            depth += 1
        elif ch == ")":
            depth -= 1
        cur.append(ch)
    part = "".join(cur).strip()
    if part:
        res.append(part)
    return res

def parse_type_and_name(raw: str) -> Tuple[str, str, Optional[int]]:
    """Parses a C-style declaration into (type, name, array_size)."""
    raw = normalize_ws(raw).rstrip(";")

    # Check for array syntax: type name[N]
    array_match = re.match(r"^(.*?)\s+([\*]*[A-Za-z_]\w*)\s*\[\s*(\d*)\s*\]$", raw)
    if array_match:
        ctype = array_match.group(1).strip()
        name = array_match.group(2).strip()
        array_len = int(array_match.group(3)) if array_match.group(3) else None
        return ctype, name, array_len

    # Standard declaration: type name
    m = re.match(r"^(.*?)(?:\s+)([\*]*[A-Za-z_]\w*)$", raw)
    if not m:
        return raw, "", None # Likely just a type (e.g., in a function pointer arg)
    ctype = m.group(1).strip()
    name = m.group(2).strip()
    return ctype, name, None

def parse_arg(arg_raw: str) -> object:
    """Parses a raw argument string into an ArgDef or FuncPtrArg."""
    # Check for function pointer: ret_type (*name)(args)
    fp = re.match(r"^(.*?)\(\s*\*\s*([A-Za-z_]\w*)\s*\)\s*\((.*)\)\s*$", arg_raw.strip())
    if fp:
        ret_str = normalize_ws(fp.group(1))
        name = fp.group(2).strip()
        arg_blob = fp.group(3).strip()

        ret_is_const = bool(re.search(r"\bconst\b", ret_str))
        ret_ptr_level = ret_str.count("*")
        ret_ctype = normalize_ws(re.sub(r"\bconst\b", "", ret_str).replace("*", " "))

        fp_args: List[ArgDef] = [parse_simple_arg(a) for a in split_args(arg_blob)]

        return FuncPtrArg(
            raw=arg_raw, name=name,
            ret_ctype=ret_ctype, ret_is_const=ret_is_const, ret_ptr_level=ret_ptr_level,
            args=fp_args,
        )
    return parse_simple_arg(arg_raw)

def parse_simple_arg(arg_raw: str) -> ArgDef:
    """Parses a simple argument string (not a function pointer)."""
    ctype_part, name_part, _ = parse_type_and_name(arg_raw)

    is_const = False
    ptr_level = 0

    # Extract constness from the type part
    if re.search(r"\bconst\b", ctype_part):
        is_const = True
        ctype_part = re.sub(r"\bconst\b", "", ctype_part).strip()

    # Count pointer levels from the type part
    ptr_level += ctype_part.count("*")
    ctype_part = ctype_part.replace("*", "").strip()

    # Count pointer levels from the name part (e.g., `**value`)
    # This handles cases like `char **value` where `name_part` might be `**value`
    # after `parse_type_and_name` extracts `value` as the identifier.
    temp_name = name_part
    while temp_name.startswith('*'):
        ptr_level += 1
        temp_name = temp_name[1:]
    name = temp_name # The actual identifier name without leading asterisks

    # Normalize the base C type
    base_ctype = normalize_ws(ctype_part)

    return ArgDef(raw=arg_raw, ctype=base_ctype, name=name, is_const=is_const, ptr_level=ptr_level)

def parse_header(content: str) -> Tuple[List[MacroDef], List[EnumDef], List[StructDef], List[FunctionDef]]:
    """Top-level parser for the entire header file content."""
    macros, enums, structs, funcs = [], [], [], []
    lines = content.splitlines()
    i = 0
    pending_comments: List[str] = []
    in_block_comment = False

    # Preprocessor state
    defined_symbols = {"WIN32"}
    if_stack = []  # List of [is_active, has_been_active]

    while i < len(lines):
        line = lines[i]

        # --- Preprocessor Logic ---
        # Handle #ifdef
        m = re.match(r'^\s*#ifdef\s+(\w+)', line)
        if m:
            sym = m.group(1)
            is_active = sym in defined_symbols
            # If parent is skipping, we must skip this block too
            if if_stack and not if_stack[-1][0]:
                is_active = False
            if_stack.append([is_active, is_active])
            i += 1
            continue

        # Handle #ifndef
        m = re.match(r'^\s*#ifndef\s+(\w+)', line)
        if m:
            sym = m.group(1)
            is_active = sym not in defined_symbols
            # If parent is skipping, we must skip this block too
            if if_stack and not if_stack[-1][0]:
                is_active = False
            if_stack.append([is_active, is_active])
            i += 1
            continue

        # Handle #else
        m = re.match(r'^\s*#else', line)
        if m:
            if if_stack:
                # If parent is skipping, we continue skipping
                parent_skipping = len(if_stack) > 1 and not if_stack[-2][0]
                if parent_skipping:
                    if_stack[-1][0] = False
                else:
                    # Toggle based on previous state in this chain
                    was_active = if_stack[-1][1]
                    if_stack[-1][0] = not was_active
                    if_stack[-1][1] = True
            i += 1
            continue

        # Handle #endif
        m = re.match(r'^\s*#endif', line)
        if m:
            if if_stack:
                if_stack.pop()
            i += 1
            continue

        # Check if we are in a skipped block
        if if_stack and not if_stack[-1][0]:
            i += 1
            continue
        # --------------------------

        # Skip empty lines
        if not line.strip():
            i += 1
            continue

        # Collect comments
        if in_block_comment:
            pending_comments.append(line)
            if "*/" in line:
                in_block_comment = False
            i += 1
            continue

        if re.match(r"^\s*/\*", line):
            pending_comments.append(line)
            if "*/" not in line:
                in_block_comment = True
            i += 1
            continue

        if re.match(r"^\s*//", line):
            pending_comments.append(line)
            i += 1
            continue

        # --- Parse Language Constructs ---

        # #define macro
        md = re.match(r"^\s*#define\s+([A-Za-z_]\w*)\s+(.*)$", line)
        if md:
            macros.append(MacroDef(md.group(1), md.group(2).strip(), pending_comments))
            pending_comments = []
            i += 1
            continue

        # typedef enum { ... } Name;
        if re.match(r"^\s*typedef\s+enum\b", line):
            comments = pending_comments
            pending_comments = []
            block = [line]
            i += 1
            while i < len(lines) and not re.search(r"}\s*[A-Za-z_]\w*\s*;", lines[i-1]):
                block.append(lines[i])
                i += 1

            full = "\n".join(block)
            name_m = re.search(r"}\s*([A-Za-z_]\w*)\s*;", full)
            enum_name = name_m.group(1) if name_m else "UnnamedEnum"
            body_m = re.search(r"{(.*?)}", full, flags=re.S)
            items: List[EnumItem] = []
            if body_m:
                for raw_item_line in body_m.group(1).splitlines():
                    s = raw_item_line.strip()
                    if not s or s.startswith("//"):
                        continue

                    cmt = ""
                    if "//" in s:
                        s, cmt = s.split("//", 1)

                    s = s.strip().rstrip(',')

                    if not s:
                        continue

                    m = re.match(r"^([A-Za-z_]\w*)(?:\s*=\s*(.+))?$", s)
                    if m:
                        items.append(EnumItem(m.group(1), m.group(2).strip() if m.group(2) else None, cmt.strip()))
            enums.append(EnumDef(enum_name, items, comments))
            continue

        # typedef struct { ... } Name;
        if re.match(r"^\s*typedef\s+struct\b", line):
            comments = pending_comments
            pending_comments = []
            block = [line]
            i += 1
            while i < len(lines) and not re.search(r"}\s*[A-Za-z_]\w*\s*;", lines[i-1]):
                block.append(lines[i])
                i += 1

            full = "\n".join(block)
            name_m = re.search(r"}\s*([A-Za-z_]\w*)\s*;", full)
            struct_name = name_m.group(1) if name_m else "UnnamedStruct"
            body_m = re.search(r"{(.*?)}", full, flags=re.S)
            fields: List[StructField] = []
            if body_m:
                for raw_field_line in body_m.group(1).splitlines():
                    s = raw_field_line.strip()
                    if not s or s.startswith("//"): continue
                    cmt = ""
                    if "//" in s:
                        s, cmt = s.split("//", 1)
                        s = s.strip()
                        cmt = cmt.strip()
                    if not s.endswith(";"): continue

                    ctype, name, arr_len = parse_type_and_name(s)

                    # Handle pointer attached to name
                    while name.startswith('*'):
                        name = name[1:]
                        ctype += '*'

                    is_const = bool(re.search(r"\bconst\b", ctype))
                    ptr_level = ctype.count("*")
                    base_ctype = normalize_ws(re.sub(r"\bconst\b", "", ctype).replace("*", " "))
                    fields.append(StructField(ctype=base_ctype, name=name, is_const=is_const, ptr_level=ptr_level, array_len=arr_len, comment=cmt))
            structs.append(StructDef(struct_name, fields, comments))
            continue

        # struct Name { ... };
        struct_decl_m = re.match(r"^\s*struct\s+([A-Za-z_]\w*)", line)
        if struct_decl_m and not line.strip().endswith(";"):
            comments = pending_comments
            pending_comments = []
            block = [line]
            struct_name = struct_decl_m.group(1)
            i += 1
            while i < len(lines) and not re.search(r"}\s*;", lines[i-1]):
                block.append(lines[i])
                i += 1

            full = "\n".join(block)
            body_m = re.search(r"{(.*?)}", full, flags=re.S)
            fields = []
            if body_m:
                for raw_field_line in body_m.group(1).splitlines():
                    s = raw_field_line.strip()
                    if not s or s.startswith("//"): continue
                    cmt = ""
                    if "//" in s:
                        s, cmt = s.split("//", 1)
                        s = s.strip()
                        cmt = cmt.strip()
                    if not s.endswith(";"): continue

                    ctype, name, arr_len = parse_type_and_name(s)

                    # Handle pointer attached to name
                    while name.startswith('*'):
                        name = name[1:]
                        ctype += '*'

                    is_const = bool(re.search(r"\bconst\b", ctype))
                    ptr_level = ctype.count("*")
                    base_ctype = normalize_ws(re.sub(r"\bconst\b", "", ctype).replace("*", " "))
                    fields.append(StructField(ctype=base_ctype, name=name, is_const=is_const, ptr_level=ptr_level, array_len=arr_len, comment=cmt))
            structs.append(StructDef(struct_name, fields, comments))
            continue

        # SE_DLL_API function declaration
        if "SE_DLL_API" in line:
            comments = pending_comments
            pending_comments = []
            proto_lines = [line]
            while ";" not in proto_lines[-1] and i < len(lines) - 1:
                i += 1
                proto_lines.append(lines[i])
            proto = normalize_ws(" ".join(proto_lines))

            fm = re.match(r"^.*SE_DLL_API\s+(.*?)\s*([A-Za-z_]\w*)\s*\((.*)\)\s*;$", proto)
            if fm:
                ret_str, name, arg_blob = fm.groups()
                ret_is_const = bool(re.search(r"\bconst\b", ret_str))
                ret_ptr_level = ret_str.count("*")
                ret_ctype = normalize_ws(re.sub(r"\bconst\b", "", ret_str).replace("*", " "))

                args = [parse_arg(a) for a in split_args(arg_blob)]
                funcs.append(FunctionDef(
                    name=name, ret_ctype=ret_ctype, ret_is_const=ret_is_const, ret_ptr_level=ret_ptr_level,
                    args=args, comments=comments
                ))
            i += 1
            continue

        # If we reach here, the line was not parsed, so clear pending comments and advance
        pending_comments = []
        i += 1

    return macros, enums, structs, funcs


# --- C# Code Generation Logic ---

def cs_delegate_arg_decl(arg: ArgDef, index: int) -> str:
    """Generates the C# declaration for a delegate argument."""
    arg_name = arg.name if arg.name else f"arg{index}" # Assign default name if empty
    if arg.ptr_level >= 1:
        return f"IntPtr {arg_name}"
    return f"{map_base_type_to_cs(arg.ctype)} {arg_name}"

def map_base_type_to_cs(ctype: str) -> str:
    """Maps a clean C base type to its C# equivalent."""
    return BASE_TYPE_MAP.get(normalize_ws(ctype), ctype)

def cs_type_for_return(ret_ctype: str, ret_is_const: bool, ret_ptr_level: int) -> str:
    """Determines the C# type for a function return value."""
    # Rule: const char* -> IntPtr
    if ret_is_const and ret_ptr_level == 1 and normalize_ws(ret_ctype) == "char":
        return "IntPtr"
    # Rule: any other pointer -> IntPtr
    if ret_ptr_level > 0:
        return "IntPtr"
    return map_base_type_to_cs(ret_ctype)

def cs_type_for_struct_field(field: StructField) -> str:
    """Determines the C# type for a struct field."""
    if field.ptr_level > 0:
        return "IntPtr"
    return map_base_type_to_cs(field.ctype)

def cs_arg_decl(arg: ArgDef, all_struct_names: Set[str]) -> str:
    """Generates the C# declaration for a function argument."""
    base_cs_type = map_base_type_to_cs(arg.ctype)
    is_struct_type = base_cs_type in all_struct_names

    # Rule: `**` -> `out IntPtr` (with special case for string arrays)
    if arg.ptr_level >= 2:
        if arg.is_const and normalize_ws(arg.ctype) == 'char':
            return f"[In] string[] {arg.name}"
        return f"out IntPtr {arg.name}"

    # Rule: `void*` -> `IntPtr`
    if arg.ptr_level == 1 and normalize_ws(arg.ctype) == 'void':
        return f"IntPtr {arg.name}"

    # Rule: `*` -> `out` (or `in` if const)
    if arg.ptr_level == 1:
        if arg.is_const:
            # `const char*` -> `string`
            if normalize_ws(arg.ctype) == 'char':
                return f"string {arg.name}"
            # `const T*` -> `in T`
            return f"in {base_cs_type} {arg.name}"
        else:
            # `T*` -> `out T`
            return f"out {base_cs_type} {arg.name}"

    # No pointer (pass by value)
    if arg.ptr_level == 0:
        # Rule: `const T` (for structs) -> `in T`
        if arg.is_const and is_struct_type:
            return f"in {base_cs_type} {arg.name}"

        if base_cs_type == 'bool':
            return f"[MarshalAs(UnmanagedType.I1)] bool {arg.name}"

        return f"{base_cs_type} {arg.name}"

def generate_cs(
    macros: List[MacroDef], enums: List[EnumDef], structs: List[StructDef], funcs: List[FunctionDef],
    namespace_name: str, class_name: str, dll_name: str
) -> str:
    """Assembles the final C# wrapper string from the parsed AST."""
    out: List[str] = []

    # Header
    out.append("/*")
    out.append(" * esmini - Environment Simulator Minimalistic")
    out.append(" * https://github.com/esmini/esmini")
    out.append(" *")
    out.append(" * This Source Code Form is subject to the terms of the Mozilla Public")
    out.append(" * License, v. 2.0. If a copy of the MPL was not distributed with this")
    out.append(" * file, You can obtain one at https://mozilla.org/MPL/2.0/.")
    out.append(" *")
    out.append(" * Copyright (c) partners of Simulation Scenarios")
    out.append(" * https://sites.google.com/view/simulationscenarios")
    out.append(" */")
    out.append("")
    out.append("/*")
    out.append(" * This module provides a generic C# interface/wrapper to the esminiLib shared library")
    out.append(" * simply mirroring the interface in terms of datatypes and functions")
    out.append(f" * Generated on: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    out.append(" */")
    out.append("")
    out.append("using System;")
    out.append("using System.Runtime.InteropServices;")
    out.append("")
    out.append(f"namespace {namespace_name}")
    out.append("{")

    all_struct_names = {s.name[3:] for s in structs}

    # --- Generate Macros ---
    if macros:
        out.append("    public static class EsminiDefines")
        out.append("    {")
        for m in macros:
            out.extend(to_xml_doc_lines(m.comments, "        "))
            val = m.value.strip()
            if re.match(r"^(0x[0-9a-fA-F]+|\d+)$", val):
                cs_type = "uint" if val.lower().startswith("0x") else "int"
                out.append(f"        public const {cs_type} {m.name} = {val};")
            else:
                escaped = val.replace("\\", "\\\\").replace("\"", "\\\"")
                out.append(f"        public const string {m.name} = \"{escaped}\";")
            out.append("")
        out.pop() # remove last empty line
        out.append("    }")
        out.append("")

    # --- Generate Enums ---
    for e in enums:
        out.extend(to_xml_doc_lines(e.comments, "    "))
        out.append(f"    public enum {e.name}")
        out.append("    {")
        for item in e.items:
            if item.comment:
                out.append(f"        /// <summary>{item.comment}</summary>")
            line = f"        {item.name}"
            if item.value is not None:
                line += f" = {item.value}"
            line += ","
            out.append(line)
        out.append("    }")
        out.append("")

    # --- Generate Structs ---
    for s in structs:
        out.extend(to_xml_doc_lines(s.comments, "    "))
        out.append("    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]")
        out.append(f"    public struct {s.name}")
        out.append("    {")
        for f in s.fields:
            if f.comment:
                out.append(f"        /// <summary>{f.comment}</summary>")

            cstype = cs_type_for_struct_field(f)

            if f.array_len is not None:
                out.append(f"        [MarshalAs(UnmanagedType.ByValArray, SizeConst = {f.array_len})]")
                out.append(f"        public {cstype}[] {f.name};")
            elif cstype == 'bool':
                out.append(f"        [MarshalAs(UnmanagedType.I1)]")
                out.append(f"        public bool {f.name};")
            else:
                out.append(f"        public {cstype} {f.name};")
        out.append("    }")
        out.append("")

    # --- Generate Native Class ---
    out.append(f"    public static partial class {class_name}")
    out.append("    {")
    out.append(f"        private const string NativeLibrary = \"{dll_name}\";")
    out.append("")

    # --- Generate Delegates ---
    generated_delegates: Dict[str, str] = {}
    for fn in funcs:
        for arg in fn.args:
            if isinstance(arg, FuncPtrArg):
                # Rule: SE_Register*Callback -> *Callback
                if fn.name.startswith("SE_Register") and fn.name.endswith("Callback"):
                    core_name = fn.name[len("SE_Register"): -len("Callback")]
                    dname = f"{core_name}Callback"
                else:
                    dname = f"{fn.name}_{arg.name}Delegate"

                if dname in generated_delegates: continue
                generated_delegates[dname] = dname

                ret_cs = cs_type_for_return(arg.ret_ctype, arg.ret_is_const, arg.ret_ptr_level) # Delegate return type
                dargs = ", ".join(cs_delegate_arg_decl(a, i) for i, a in enumerate(arg.args)) # Delegate arguments

                out.append("        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]")
                out.append(f"        public delegate {ret_cs} {dname}({dargs});")
                out.append("")

    # --- Generate Functions ---
    for fn in funcs:
        out.extend(to_xml_doc_lines(fn.comments, "        "))
        ret_cs = cs_type_for_return(fn.ret_ctype, fn.ret_is_const, fn.ret_ptr_level)

        cs_args = []
        for arg in fn.args:
            if isinstance(arg, FuncPtrArg):
                if fn.name.startswith("SE_Register") and fn.name.endswith("Callback"):
                    core_name = fn.name[len("SE_Register"): -len("Callback")]
                    dname = f"{core_name}Callback"
                else:
                    dname = f"{fn.name}_{arg.name}Delegate"
                cs_args.append(f"{dname} {arg.name}")
            elif isinstance(arg, ArgDef):
                cs_args.append(cs_arg_decl(arg, all_struct_names))

        out.append(f"        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]")
        if ret_cs == 'bool':
             out.append(f"        [return: MarshalAs(UnmanagedType.I1)]")
        out.append(f"        public static extern {ret_cs} {fn.name}({', '.join(cs_args)});")
        out.append("")

    out.pop() # remove last empty line
    out.append("    }")
    out.append("}")
    return "\n".join(out)


# --- Main Execution ---

def main():
    """Main entry point for the script."""
    parser = argparse.ArgumentParser(description="Generate C# wrapper from a C++ header file.")
    parser.add_argument("input_hpp", help="Path to the input C++ header file (e.g., esminiLib.hpp)")
    parser.add_argument("output_cs", help="Path for the generated C# output file")
    parser.add_argument("--namespace", default="Esmini", dest="namespace_name", help="C# namespace for the generated code")
    parser.add_argument("--class", default="EsminiNative", dest="class_name", help="C# static class name for native functions")
    parser.add_argument("--dll", default="esminiLib", dest="dll_name", help="Name of the native DLL to import")
    args = parser.parse_args()

    try:
        with open(args.input_hpp, "r", encoding="utf-8") as f:
            content = f.read()
    except FileNotFoundError:
        print(f"Error: Input file not found at '{args.input_hpp}'")
        return

    print("Parsing header file...")
    macros, enums, structs, funcs = parse_header(content)
    print(f"Found: {len(macros)} macros, {len(enums)} enums, {len(structs)} structs, {len(funcs)} functions.")

    print("Generating C# wrapper...")
    cs_code = generate_cs(
        macros=macros, enums=enums, structs=structs, funcs=funcs,
        namespace_name=args.namespace_name, class_name=args.class_name, dll_name=args.dll_name
    )

    try:
        with open(args.output_cs, "w", encoding="utf-8", newline="\n") as f:
            f.write(cs_code)
        print(f"Successfully generated C# wrapper: {args.output_cs}")
    except IOError as e:
        print(f"Error: Could not write to output file '{args.output_cs}'. Reason: {e}")

if __name__ == "__main__":
    main()
