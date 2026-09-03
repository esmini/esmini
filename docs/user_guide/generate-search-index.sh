#!/usr/bin/env bash
set -euo pipefail

docs_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Optional first argument: output folder for the generated index (defaults to docs_dir)
case "${1:-}" in
  -h|--help)
    cat <<EOF
Usage: $(basename "${BASH_SOURCE[0]}") [OUTPUT_DIR]

Generate search-index.json from the AsciiDoc (.adoc) files in this folder.

Arguments:
  OUTPUT_DIR   Optional folder to write search-index.json to.
               Created if it does not exist. Defaults to the script's folder.

Options:
  -h, --help   Show this help message and exit.
EOF
    exit 0
    ;;
esac

output_dir="${1:-$docs_dir}"
mkdir -p "$output_dir"
output_dir="$(cd "$output_dir" && pwd)"
output_path="$output_dir/search-index.json"

json_escape() {
  local value="$1"
  value=${value//\\/\\\\}
  value=${value//\"/\\\"}
  value=${value//$'\n'/\\n}
  value=${value//$'\r'/}
  value=${value//$'\t'/ }
  printf '%s' "$value"
}

extract_title() {
  local file_name="$1"
  local content="$2"

  if [[ "$file_name" == "index.adoc" ]]; then
    printf 'Home'
    return
  fi

  if [[ "$file_name" == "toc.adoc" ]]; then
    printf 'Table of contents'
    return
  fi

  local section_title
  section_title="$(printf '%s\n' "$content" | sed -n 's/^==[[:space:]]\+//p' | head -n 1)"
  if [[ -n "$section_title" ]]; then
    printf '%s' "$section_title"
    return
  fi

  local doc_title
  doc_title="$(printf '%s\n' "$content" | sed -n 's/^=[[:space:]]\+//p' | head -n 1)"
  if [[ -n "$doc_title" ]]; then
    printf '%s' "$doc_title"
    return
  fi

  printf '%s' "${file_name%.adoc}"
}

sanitize_text() {
  perl -0777 -pe '
    s/<script\b[^>]*>.*?<\/script>//gsi;
    s/<style\b[^>]*>.*?<\/style>//gsi;
    s/<[^>]+>/ /g;
    s/&(?:nbsp|#160);/ /g;
    s/&amp;/\&/g;
    s/&lt;/</g;
    s/&gt;/>/g;
    s/&quot;/"/g;
    s/&#39;/'\''/g;
    s/\s+/ /g;
    s/^\s+|\s+$//g;
  '
}

render_text() {
  local file_path="$1"
  local file_dir
  local file_name
  file_dir="$(dirname "$file_path")"
  file_name="$(basename "$file_path")"

  (
    cd "$file_dir"
    sed '/^include::nav\.adoc_\[\]$/d' "$file_name" |
      asciidoctor --embedded --attribute toc! --out-file - -
  )
}

shopt -s nullglob
files=("$docs_dir/index.adoc" "$docs_dir/toc.adoc")
for file_path in "$docs_dir"/*.adoc; do
  file_name="$(basename "$file_path")"
  case "$file_name" in
    index.adoc|toc.adoc|search.adoc|monolith.adoc)
      continue
      ;;
  esac
  files+=("$file_path")
done
IFS=$'\n' read -r -d '' -a files < <(printf '%s\n' "${files[@]}" | sort -u && printf '\0')

{
  printf '[\n'
  first=1

  for file_path in "${files[@]}"; do
    [[ -f "$file_path" ]] || continue

    file_name="$(basename "$file_path")"
    raw="$(cat "$file_path")"
    title="$(extract_title "$file_name" "$raw")"
    text="$(render_text "$file_path" | sanitize_text)"
    href="${file_name%.adoc}.html"

    escaped_title="$(json_escape "$title")"
    escaped_href="$(json_escape "$href")"
    escaped_text="$(json_escape "$text")"

    if [[ $first -eq 0 ]]; then
      printf ',\n'
    fi
    first=0

    printf '  {"title":"%s","href":"%s","text":"%s"}' "$escaped_title" "$escaped_href" "$escaped_text"
  done

  printf '\n]\n'
} > "$output_path"

echo "Wrote $output_path"