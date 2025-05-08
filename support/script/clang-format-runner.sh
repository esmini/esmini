
# Detect OS and set the appropriate clang-format command
if [[ "$OSTYPE" == "msys"* || "$OSTYPE" == "cygwin"* || "$OSTYPE" == "win32" ]]; then
    CMD="clang-format"
else
    CMD="clang-format-15"
fi

# Pass all arguments to the selected clang-format command
exec "$CMD" "$@"

