""" external/rules/models.bzl """

def _get_models_impl(rctx):
    rctx.symlink(
        Label("@esmini//support/bazel/external/target:BUILD.models.bazel"),
        "BUILD.bazel",
    )

    download = rctx.execute(["curl", "-k", "-O", "https://esmini.asuscomm.com/AICLOUD779364751/models/models.7z"])
    if download.return_code != 0:
        fail("Failed to download the archive.")

    extract = rctx.execute(["7z", "x", "models.7z", "-o."])
    if extract.return_code != 0:
        fail("Failed to extract the archive.")

    rctx.delete("models.7z")

get_models = repository_rule(
    implementation = _get_models_impl,
)
