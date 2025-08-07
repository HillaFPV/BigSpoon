from typing import Union
from fastapi import FastAPI
import subprocess

app = FastAPI()


@app.get("/")
def read_root():
    subprocess.run(["ffplay", "-autoexit", "C:\\Users\\HillaFPV\\Drone Hangar 3.0\\Applause (Basic).mp3"])
    return "Hello, poopie bottom!"



@app.get("/items/{item_id}")
def read_item(item_id: int, q: Union[str, None] = None):
    return {"item_id": item_id, "q": q}