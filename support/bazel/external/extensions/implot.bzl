""" external/extensions/implot.bzl """

load("@esmini//support/bazel/external/rules:implot.bzl", "get_implot")

def _implot_impl(module_ctx):
    for module in module_ctx.modules:
        for attr in module.tags.base:
            attrs = {
                key: getattr(attr, key)
                for key in dir(attr)
                if not key.startswith("_")
            }
            get_implot(**attrs)

_implot_attrs = {
    "name": attr.string(default = "implot"),
}

implot = module_extension(
    implementation = _implot_impl,
    tag_classes = {
        "base": tag_class(
            attrs = _implot_attrs,
        ),
    },
)
