import sys
import json
import random

from openai import OpenAI
from pathlib import Path
import base64
import json

OPENAI_API_KEY = ""
OPENAI_BASE_URL = ""

client = OpenAI(api_key=OPENAI_API_KEY, base_url=OPENAI_BASE_URL)

# Evaluation prompt
FINDER_PROMPT_original = """
The input image is a generated Jiangnan garden from top view.
Based on the user input, please find appropriate key positions of the image.
And just return the key points, in format:
{
    [x1, y1,"reason for the selection"], 
    [x2, y2,"reason for the selection"], 
    [x3, y3, "reason for the selection"]
    ...
}
the x and y are the coordinates of the key points in range of 0-1, keeping 3 decimal places.

Here is the user input: "*{user_instruction}*".

"""


FINDER_PROMPT = """
The input image is a top-view rendering of a traditional Jiangnan garden.
Your task is to carefully analyze the scene and determine the most appropriate key positions based on the following user instruction:
"*{user_instruction}*".

Please follow these rules strictly:
1. Identify meaningful locations that best satisfy the user's instruction (e.g., scenic centers, architectural highlights, water features, pathways, etc.).
2. Return only the key points that are directly relevant to the instruction.
3. Each key point must include a short, clear reasoning for why it was selected.
4. The (x, y) coordinates must be normalized to the range [0, 1] and rounded to 3 decimal places.
5. Do not include any extra explanations or text outside the required JSON format.

Output format (strict JSON):
{
  "points": [
    {"x": 0.523, "y": 0.742, "reason": "near the pavilion entrance"},
    {"x": 0.312, "y": 0.585, "reason": "central pond area"},
    {"x": 0.851, "y": 0.430, "reason": "bridge intersection"}
  ]
}
"""

def encode_image(image_path):
    """Encodes an image as base64 string."""
    with open(image_path, "rb") as f:
        return base64.b64encode(f.read()).decode("utf-8")

def find_key_points(user_input, image_path=None):
    debug = False
    if not debug:
        prompt = FINDER_PROMPT.replace("*{user_instruction}*", str(user_input))
        base64_image = encode_image(image_path)
        response = get_vlm_response(prompt, base64_image)
    else:
        response = {
            "status": "ok",
            "points": [
                {"x": 0.02, "y": 0.02, "reason": "null"},
                {"x": 0.02, "y": 0.02, "reason": "null"},
                {"x": 0.02, "y": 0.02, "reason": "null"}
            ]
        }
    return response

def get_vlm_response(input_prompts: str, base64_image: str):

    response = client.responses.create(
        model="gpt-5",
        input=[
            {
                "role": "user",
                "content": [
                    { "type": "input_text", "text": input_prompts },
                    {
                        "type": "input_image",
                        "image_url": {
                            "url": f"data:image/png;base64,{base64_image}"
                        }
                    }
                ]
            }
        ]
    )

    text_response = None
    for output in response.output:
        if output.type == "message":
            for content_part in output.content:
                if content_part.type == "output_text":
                    text_response = content_part.text
                    break  # found it, no need to continue
        if text_response:
            break
    # will the print affect output = proc.StandardOutput.ReadToEnd() ?
    if text_response:
        text_response["status"] = "ok"
    else:
        text_response["status"] = None
    return text_response

def main():
    #data = json.load(sys.stdin)
    data = {
        "prompt": "Find the most interesting points in the Garden, I want to take a photo of them.",
        "image_path": r"E:\Scientific Research\Layout\ControllableLandscape\Conlan\Assets\SceneGen\Image_Landscape\top_view.png"
    }
    prompt = data.get("prompt", "")
    image_path = data.get("image_path", None)

    try:
        res = find_key_points(prompt, image_path)
        print(json.dumps(res))
    except Exception as e:
        print(json.dumps({"status":"error","error":str(e)}))
        sys.exit(1)

if __name__ == "__main__":
    main()