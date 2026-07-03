import os
import re
from pathlib import Path

import torch

model_path = Path(__file__).resolve().parent.parent / "models"


def sanitize_token(token):
    if token is None:
        return None
    if isinstance(token, (list, tuple)):
        token = ",".join(str(x) for x in token)
    if isinstance(token, dict):
        token = "+".join(f"{k}={v}" for k, v in sorted(token.items()))
    token = str(token)
    token = token.replace(" ", "_").replace("/", "_").replace("\\", "_").replace(":", "-")
    token = re.sub(r"[^A-Za-z0-9_.-]+", "_", token)
    token = re.sub(r"_+", "_", token).strip("_.-")
    return token[:120] if token else None


def extract_model_metadata(checkpoint):
    if not isinstance(checkpoint, dict):
        return {}

    metadata = {}
    metadata["version"] = checkpoint.get("version")
    metadata["epoch"] = checkpoint.get("epoch") or checkpoint.get("last_epoch")
    metadata["fitness"] = checkpoint.get("best_fitness")

    train_args = checkpoint.get("train_args") or {}
    metadata["train_args"] = train_args

    model_obj = checkpoint.get("model")
    if model_obj is not None:
        yaml_obj = getattr(model_obj, "yaml", None)
        if isinstance(yaml_obj, dict):
            metadata["arch"] = yaml_obj.get("yaml_file") or yaml_obj.get("scale")
        elif isinstance(yaml_obj, str):
            metadata["arch"] = yaml_obj
        elif hasattr(model_obj, "__class__"):
            metadata["arch"] = model_obj.__class__.__name__

        if hasattr(model_obj, "names") and getattr(model_obj, "names") is not None:
            names = getattr(model_obj, "names")
            if isinstance(names, (list, tuple, dict)):
                metadata["classes"] = len(names)
            else:
                metadata["names"] = names

    return metadata


def normalize_dataset_name(value):
    if not value:
        return None
    value = str(value).strip()
    if value.endswith(".yaml") or value.endswith(".yml"):
        value = str(Path(value).parent.name or Path(value).stem)
    elif os.path.exists(value):
        value = str(Path(value).resolve().parent.name or Path(value).stem)
    elif "/" in value or "\\" in value:
        value = Path(value).name
    return sanitize_token(value)


def unique_model_name(base_stem, current_file):
    candidate_name = f"{base_stem}{current_file.suffix}"
    candidate_path = model_path / candidate_name
    if not candidate_path.exists() or candidate_path == current_file:
        return candidate_name

    suffix_index = 2
    while True:
        candidate_name = f"{base_stem}_v{suffix_index}{current_file.suffix}"
        candidate_path = model_path / candidate_name
        if not candidate_path.exists() or candidate_path == current_file:
            return candidate_name
        suffix_index += 1


def build_new_model_name(original_path, checkpoint):
    metadata = extract_model_metadata(checkpoint)

    pieces = []

    arch = metadata.get("arch")
    if arch:
        arch = sanitize_token(arch)
        if arch:
            if arch.endswith(".yaml"):
                arch = Path(arch).stem
            pieces.append(arch)

    if not pieces:
        train_args = metadata.get("train_args", {})
        model_name = train_args.get("model")
        if model_name:
            pieces.append(sanitize_token(Path(str(model_name)).stem))

    dataset_token = None
    train_args = metadata.get("train_args", {})
    for candidate in (train_args.get("data"), train_args.get("project"), train_args.get("name")):
        token = normalize_dataset_name(candidate)
        if token:
            dataset_token = token
            break

    if dataset_token:
        pieces.append(dataset_token)

    if not pieces:
        pieces = [original_path.stem]

    base_stem = "_".join(pieces)
    return unique_model_name(base_stem, original_path)


def rename(dry_run=True):
    """Rename every model checkpoint file in ../models based on checkpoint metadata.

    This scans the models directory for common PyTorch checkpoint extensions and
    assigns a unique name like yolo26l_ds09.pt. If the base name already exists,
    it appends _v2, _v3, etc.

    If dry_run is True, the function only prints the proposed renames.
    """
    if not model_path.exists() or not model_path.is_dir():
        raise FileNotFoundError(f"Model path not found: {model_path}")

    checkpoint_patterns = ["*.pt", "*.pth", "*.pth.tar"]
    checkpoint_files = []
    for pattern in checkpoint_patterns:
        checkpoint_files.extend(model_path.glob(pattern))

    renamed = []
    for model_file in sorted(set(checkpoint_files)):
        try:
            checkpoint = torch.load(model_file, map_location="cpu", weights_only=False)
        except Exception as exc:
            print(f"Skipping {model_file.name}: failed to load checkpoint ({exc})")
            continue

        new_name = build_new_model_name(model_file, checkpoint)
        if new_name == model_file.name:
            continue

        new_path = model_file.with_name(new_name)
        print(f"Rename: {model_file.name} -> {new_name}")
        renamed.append((model_file.name, new_name))
        if not dry_run:
            model_file.rename(new_path)

    return renamed


def get_model_data(model_name="bestb.pt"):
    checkpoint_path = model_path / model_name
    checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=False)

    print(type(checkpoint))
    if isinstance(checkpoint, dict):
        print(checkpoint.keys())

    sd = checkpoint["model"].state_dict() if hasattr(checkpoint.get("model", checkpoint), "state_dict") else checkpoint

    print("SPECIFIC RELEVANT DATA:")
    print(checkpoint.get("version"))

    return extract_model_metadata(checkpoint)


def main():
    # dry_run states for wether the run can (=False) or cannot (=True) rename the files
    rename(dry_run=False)


if __name__ == "__main__":
    main()