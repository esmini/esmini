""" external/extensions/osg.bzl """

load("@esmini//support/bazel/external/rules:osg.bzl", "get_osg")

def _osg_impl(module_ctx):
    for module in module_ctx.modules:
        for attr in module.tags.base:
            attrs = {
                key: getattr(attr, key)
                for key in dir(attr)
                if not key.startswith("_")
            }
            get_osg(**attrs)

_osg_attrs = {
    "name": attr.string(default = "osg"),
}

osg = module_extension(
    implementation = _osg_impl,
    tag_classes = {
        "base": tag_class(
            attrs = _osg_attrs,
        ),
    },
)
