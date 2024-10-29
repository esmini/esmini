""" external/extensions/sumo.bzl """

load("@esmini//support/bazel/external/rules:sumo.bzl", "get_sumo")

def _sumo_impl(module_ctx):
    for module in module_ctx.modules:
        for attr in module.tags.base:
            attrs = {
                key: getattr(attr, key)
                for key in dir(attr)
                if not key.startswith("_")
            }
            get_sumo(**attrs)

_sumo_attrs = {
    "name": attr.string(default = "sumo"),
}

sumo = module_extension(
    implementation = _sumo_impl,
    tag_classes = {
        "base": tag_class(
            attrs = _sumo_attrs,
        ),
    },
)
