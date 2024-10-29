""" external/extensions/osi.bzl """

load("@esmini//support/bazel/external/rules:osi.bzl", "get_osi")

def _osi_impl(module_ctx):
    for module in module_ctx.modules:
        for attr in module.tags.base:
            attrs = {
                key: getattr(attr, key)
                for key in dir(attr)
                if not key.startswith("_")
            }
            get_osi(**attrs)

_osi_attrs = {
    "name": attr.string(default = "osi"),
}

osi = module_extension(
    implementation = _osi_impl,
    tag_classes = {
        "base": tag_class(
            attrs = _osi_attrs,
        ),
    },
)
