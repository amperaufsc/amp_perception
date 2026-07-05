import os
from pathlib import Path
from huggingface_hub import hf_hub_download, HfApi
from ultralytics import YOLO
from dotenv import load_dotenv

load_dotenv()

HF_USERNAME = os.getenv("HF_USERNAME")
REPO_NAME = os.getenv("REPO_NAME")
HF_TOKEN_READ = os.getenv("HF_TOKEN_READ")
HF_TOKEN_WRITE = os.getenv("HF_TOKEN_WRITE")
REPO_ID = os.getenv("REPO_ID")
DOWNLOAD_DIR = Path(__file__).resolve().parent.parent / "models"
DOWNLOAD_DIR.mkdir(parents=True, exist_ok=True)

SUPPORTED_MODEL_EXTENSIONS = {".pt", ".pth", ".pth.tar", ".onnx", ".engine", ".tflite", ".pb", ".yaml", ".yml"}

api = HfApi(token=HF_TOKEN_READ)


def is_model_file(filename: str) -> bool:
    return Path(filename).suffix.lower() in SUPPORTED_MODEL_EXTENSIONS


def list_repo_files(repo_id: str):
    try:
        return api.list_repo_files(repo_id=repo_id)
    except Exception as exc:
        raise RuntimeError(f"Failed to list files from repo {repo_id}: {exc}")


def download_repo_file(repo_id: str, filename: str, local_dir: Path, token: str = None) -> Path:
    dest = local_dir / filename
    dest.parent.mkdir(parents=True, exist_ok=True)
    if dest.exists():
        print(f"Already downloaded: {dest}")
        return dest

    print(f"Downloading {filename} from {repo_id} -> {dest}")
    local_path = hf_hub_download(
        repo_id=repo_id,
        filename=filename,
        token=token,
        local_dir=str(local_dir),
        local_dir_use_symlinks=False,
    ) # type: ignore
    return Path(local_path)


def download_all_models_from_repo(repo_id: str, local_dir: Path, token: str = None):
    files = list_repo_files(repo_id)
    model_files = [f for f in files if is_model_file(f)]

    if not model_files:
        raise RuntimeError(f"No supported model files found in repo {repo_id}")

    downloaded_paths = []
    for filename in sorted(model_files):
        downloaded_paths.append(download_repo_file(repo_id, filename, local_dir, token=token))

    return downloaded_paths


def load_model(path: Path):
    print(f"Loading model from {path}")
    return YOLO(str(path), task="detect")


def main():
    repo_id = REPO_ID
    if not repo_id:
        if HF_USERNAME and REPO_NAME:
            repo_id = f"{HF_USERNAME}/{REPO_NAME}"
        else:
            raise RuntimeError("HF repo ID is not configured. Set HF_USERNAME and REPO_NAME, or REPO_ID.")

    downloaded_paths = download_all_models_from_repo(repo_id, DOWNLOAD_DIR, token=HF_TOKEN_READ) # type: ignore
    print(f"Downloaded {len(downloaded_paths)} model files")

    models = []
    for path in downloaded_paths:
        if path.suffix.lower() in {".pt", ".pth", ".onnx", ".engine"}:
            models.append(load_model(path))

    print(f"Loaded {len(models)} YOLO models")
    return models


if __name__ == "__main__":
    main()
