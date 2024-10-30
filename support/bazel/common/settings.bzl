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

implot_provider = provider("implot_provider", fields = ["type"])

def _implot_impl(ctx):
    return implot_provider(type = ctx.build_setting_value)

implot = rule(
    implementation = _implot_impl,
    build_setting = config.string(flag = True),
)

####################################################################################################

osg_provider = provider("osg_provider", fields = ["type"])

def _osg_impl(ctx):
    return osg_provider(type = ctx.build_setting_value)

osg = rule(
    implementation = _osg_impl,
    build_setting = config.string(flag = True),
)

####################################################################################################

osi_provider = provider("osi_provider", fields = ["type"])

def _osi_impl(ctx):
    return osi_provider(type = ctx.build_setting_value)

osi = rule(
    implementation = _osi_impl,
    build_setting = config.string(flag = True),
)

####################################################################################################

sumo_provider = provider("sumo_provider", fields = ["type"])

def _sumo_impl(ctx):
    return sumo_provider(type = ctx.build_setting_value)

sumo = rule(
    implementation = _sumo_impl,
    build_setting = config.string(flag = True),
)

####################################################################################################