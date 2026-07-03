from huggingface_hub import hf_hub_download, HfApi
from ultralytics import YOLO
from dotenv import load_dotenv
import os

load_dotenv()

HF_USERNAME = os.getenv("HF_USERNAME")
MODEL_FILENAME = os.getenv("MODEL_FILENAME")
REPO_NAME = os.getenv("REPO_NAME")
HF_TOKEN_READ = os.getenv("HF_TOKEN_READ")
HF_TOKEN_WRITE = os.getenv("HF_TOKEN_WRITE")

api = HfApi(token=HF_TOKEN_READ)
api.whoami()

# TODO: Create a list with all the possible models to downlaod them and list them for easing their usage
# for model in os.listdir()
# Download and load model
model_path = hf_hub_download(
    repo_id=f"{HF_USERNAME}/{REPO_NAME}",
    filename=MODEL_FILENAME,
    token=HF_TOKEN_READ
)

files = api.list_repo_files(repo_id=f"{HF_USERNAME}/{REPO_NAME}")
print(files)

model = YOLO(model_path, task="detect")
model = model.export("engine")
