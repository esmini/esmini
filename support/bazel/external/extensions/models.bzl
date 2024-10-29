""" external/extensions/models.bzl """

load("@esmini//support/bazel/external/rules:models.bzl", "get_models")

def _models_impl(module_ctx):
    for module in module_ctx.modules:
        for attr in module.tags.base:
            attrs = {
                key: getattr(attr, key)
                for key in dir(attr)
                if not key.startswith("_")
            }
            get_models(**attrs)

_models_attrs = {
    "name": attr.string(default = "models"),
}

models = module_extension(
    implementation = _models_impl,
    tag_classes = {
        "base": tag_class(
            attrs = _models_attrs,
        ),
    },
)
