""" external/rules/sumo.bzl """

def _get_sumo_impl(rctx):
    rctx.symlink(
        Label("@esmini//support/bazel/external/target:BUILD.sumo.bazel"),
        "BUILD.bazel",
    )

    download = rctx.execute(["curl", "-k", "-O", "https://esmini.asuscomm.com/AICLOUD766065121/libs/sumo_linux.7z"])
    if download.return_code != 0:
        fail("Failed to download the archive.")

    extract = rctx.execute(["7z", "x", "sumo_linux.7z", "-o."])
    if extract.return_code != 0:
        fail("Failed to extract the archive.")

    rctx.delete("sumo_linux.7z")

get_sumo = repository_rule(
    implementation = _get_sumo_impl,
)
