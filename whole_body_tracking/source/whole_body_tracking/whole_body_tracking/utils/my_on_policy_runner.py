import os

from rsl_rl.env import VecEnv
from rsl_rl.runners.on_policy_runner import OnPolicyRunner

from isaaclab_rl.rsl_rl import export_policy_as_onnx

import wandb
from whole_body_tracking.utils.exporter import attach_onnx_metadata, export_motion_policy_as_onnx


def _get_obs_normalizer(runner):
    # rsl_rl versions differ: sometimes runner has obs_normalizer, sometimes alg has it, sometimes none.
    if hasattr(runner, "obs_normalizer"):
        return runner.obs_normalizer
    if hasattr(runner, "alg") and hasattr(runner.alg, "obs_normalizer"):
        return runner.alg.obs_normalizer
    return None


def _wandb_enabled():
    # Don’t depend on runner.logger_type (varies by rsl_rl version)
    return wandb.run is not None and os.environ.get("WANDB_MODE", "").lower() != "disabled"


class MyOnPolicyRunner(OnPolicyRunner):
    def save(self, path: str, infos=None):
        """Save the model and training information."""
        super().save(path, infos)

        # Never let export break training
        try:
            if not _wandb_enabled():
                return

            policy_path = path.split("model")[0]
            filename = policy_path.split("/")[-2] + ".onnx"
            normalizer = _get_obs_normalizer(self)

            export_policy_as_onnx(
                self.alg.policy,
                normalizer=normalizer,
                path=policy_path,
                filename=filename,
            )
            attach_onnx_metadata(self.env.unwrapped, wandb.run.name, path=policy_path, filename=filename)
            wandb.save(policy_path + filename, base_path=os.path.dirname(policy_path))
        except Exception as e:
            print(f"[WARN] ONNX/W&B export failed (continuing training): {e}")


class MotionOnPolicyRunner(OnPolicyRunner):
    def __init__(
        self, env: VecEnv, train_cfg: dict, log_dir: str | None = None, device="cpu", registry_name: str = None
    ):
        super().__init__(env, train_cfg, log_dir, device)
        self.registry_name = registry_name

    def save(self, path: str, infos=None):
        """Save the model and training information."""
        super().save(path, infos)

        # Never let export break training
        try:
            if not _wandb_enabled():
                return

            policy_path = path.split("model")[0]
            filename = policy_path.split("/")[-2] + ".onnx"
            normalizer = _get_obs_normalizer(self)

            export_motion_policy_as_onnx(
                self.env.unwrapped,
                self.alg.policy,
                normalizer=normalizer,
                path=policy_path,
                filename=filename,
            )
            attach_onnx_metadata(self.env.unwrapped, wandb.run.name, path=policy_path, filename=filename)
            wandb.save(policy_path + filename, base_path=os.path.dirname(policy_path))

            # link the artifact registry to this run
            if self.registry_name is not None:
                wandb.run.use_artifact(self.registry_name)
                self.registry_name = None
        except Exception as e:
            print(f"[WARN] Motion ONNX/W&B export failed (continuing training): {e}")
