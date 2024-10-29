""" external/rules/osi.bzl """

def _get_osi_impl(rctx):
    rctx.symlink(
        Label("@esmini//support/bazel/external/target:BUILD.osi.bazel"),
        "BUILD.bazel",
    )

    download = rctx.execute(["curl", "-k", "-O", "https://esmini.asuscomm.com/AICLOUD766065121/libs/osi_3_5_0_linux_glibc_2_31_gcc_7_5_0_v2.7z"])
    if download.return_code != 0:
        fail("Failed to download the archive.")

    extract = rctx.execute(["7z", "x", "osi_3_5_0_linux_glibc_2_31_gcc_7_5_0_v2.7z", "-o."])
    if extract.return_code != 0:
        fail("Failed to extract the archive.")

    rctx.delete("osi_3_5_0_linux_glibc_2_31_gcc_7_5_0_v2.7z")

get_osi = repository_rule(
    implementation = _get_osi_impl,
)
