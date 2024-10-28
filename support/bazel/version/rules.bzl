""" version/rules.bzl """

########################################################################################################################

def _generate_version_impl(ctx):
    args = ctx.actions.args()
    args.add(ctx.version_file)
    args.add_all(ctx.outputs.generated_file)

    ctx.actions.run(
        inputs = [
            ctx.version_file,
        ],
        outputs = ctx.outputs.generated_file,
        executable = ctx.executable._command,
        arguments = [args],
    )

    return [DefaultInfo(files = depset(ctx.outputs.generated_file))]

generate_version = rule(
    implementation = _generate_version_impl,
    attrs = {
        "generated_file": attr.output_list(
            mandatory = True,
        ),
        "_command": attr.label(
            cfg = "exec",
            executable = True,
            default = Label("//support/bazel/version:generate_version"),
        ),
    },
)

########################################################################################################################
