""" external/rules/osg.bzl """

def _get_osg_impl(rctx):
    rctx.symlink(
        Label("@esmini//support/bazel/external/target:BUILD.osg.bazel"),
        "BUILD.bazel",
    )

    download = rctx.execute(["curl", "-k", "-O", "https://esmini.asuscomm.com/AICLOUD766065121/libs/osg_linux_glibc_2_31_gcc_7_5_0.7z"])
    if download.return_code != 0:
        fail("Failed to download the archive.")

    extract = rctx.execute(["7z", "x", "osg_linux_glibc_2_31_gcc_7_5_0.7z", "-o."])
    if extract.return_code != 0:
        fail("Failed to extract the archive.")

    rctx.delete("osg_linux_glibc_2_31_gcc_7_5_0.7z")

get_osg = repository_rule(
    implementation = _get_osg_impl,
)
