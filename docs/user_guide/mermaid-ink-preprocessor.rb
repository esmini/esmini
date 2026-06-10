require 'asciidoctor'
require 'asciidoctor/extensions'
require 'zlib'
require 'base64'
require 'cgi'
require 'json' # Added to structure the payload correctly

Asciidoctor::Extensions.register do
  block do
    named :mermaid
    on_contexts :literal, :listing, :open
    parse_content_as :raw

    process do |parent, reader, attrs|
      mermaid_text = reader.read

      # 1. Wrap the raw text into the explicit JSON structure mermaid.ink expects
      payload_json = {
        code: mermaid_text,
        mermaid: {
          theme: attrs['theme'] || 'default'
        }
      }.to_json

      # 2. Compress the JSON string using standard Zlib raw deflate
      deflated = Zlib::Deflate.deflate(payload_json, Zlib::BEST_COMPRESSION)

      # 3. Use STRICT Base64 encoding
      strict_b64 = Base64.strict_encode64(deflated)

      # 4. URL-encode the base64 string
      safe_payload = CGI.escape(strict_b64)

      # 5. Construct the final endpoint URL
      mermaid_url = "https://mermaid.ink/svg/pako:#{safe_payload}"

      image_attrs = {
        'target' => mermaid_url,
        'alt' => attrs['title'] || 'Mermaid Diagram',
        'title' => attrs['title']
      }

      image_attrs['width'] = attrs['width'] if attrs['width']
      image_attrs['height'] = attrs['height'] if attrs['height']

      create_image_block parent, image_attrs
    end
  end
end