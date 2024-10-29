""" settings.bzl """

####################################################################################################

dyn_protobuf_provider = provider("dyn_protobuf_provider", fields = ["type"])

def _dyn_protobuf_impl(ctx):
    return dyn_protobuf_provider(type = ctx.build_setting_value)

dyn_protobuf = rule(
    implementation = _dyn_protobuf_impl,
    build_setting = config.string(flag = True),
)

####################################################################################################
