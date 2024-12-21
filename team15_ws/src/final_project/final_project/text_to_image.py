from dotenv import load_dotenv
import os
import openai
from openai import OpenAI
import torch
from diffusers import StableDiffusionPipeline
from diffusers import DiffusionPipeline
import cv2
import numpy as np
import sys
sys.path.append(f"/home/robot/anaconda3/envs/team15/lib/python3.10/site-packages/")

load_dotenv()
openai.api_key = os.getenv("OPENAI_API_KEY")

class TextToImage():
    def __init__(self):
        pass

    def extract_illustration_elements(self, raw_text):
        client = OpenAI()
        system_prompt = (
            "You are a helpful assistant that focuses on extracting the key objects "
            "or subjects from a given piece of text. The extracted list should be simple and "
            "limited to the main elements that would be illustrated."
        )
        
        user_prompt = (
            f"Here is some raw text:\n\n{raw_text}\n\n"
            "From this text, identify the main objects or subjects that should appear in a simple illustration. "
            "Keep the list short, direct, and easy to visualize. Only provide the objects, no additional context. "
            "Return them as a concise bullet point list."
        )
        
        response = client.chat.completions.create(
            model="gpt-4o-mini",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ],
            temperature=0.7
        )
        
        return response.choices[0].message.content.strip()


    def get_objects_description(self, illustration_elements):
        lines = illustration_elements.split("\n")
        # Strip dashes and extra whitespace, then join them into a single phrase
        objects_list = [line.strip("- ").strip() for line in lines if line.strip("- ").strip()]
        objects_description = " , ".join(objects_list)

        return objects_description

    def generate_line_art_image(self, objects_description: str, output_path: str = "output.png"):
        # Load a stable diffusion pipeline (pick a suitable model)
        # pipe = DiffusionPipeline.from_pretrained(
        #     "stabilityai/stable-diffusion-xl-base-1.0",
        #     torch_dtype=torch.float16
        # ).to("cuda")

        # pipe.load_lora_weights("./T2I_model/pytorch_lora_weights.safetensors")

        pipe = DiffusionPipeline.from_pretrained("yehiaserag/anime-pencil-diffusion").to("cuda")
        print(f"Post-load VRAM (allocated): {torch.cuda.memory_allocated()/1024**2:.2f} MB")
        print(f"Post-load VRAM (reserved): {torch.cuda.memory_reserved()/1024**2:.2f} MB")


        # Craft a prompt for a simple black-and-white line drawing
        prompt = (
            f"A simple black and white line drawing of full {objects_description}, "
            "minimalistic, thin, clean outlines, no shading, no background, pencil sketch style."
            "And the objects should be complete."
        )

        # Generate the image
        image = pipe(prompt, num_inference_steps=50, guidance_scale=7.5, height=304, width=456).images[0]

        print(f"Final VRAM (allocated): {torch.cuda.memory_allocated()/1024**2:.2f} MB")
        print(f"Final VRAM (reserved): {torch.cuda.memory_reserved()/1024**2:.2f} MB")
        print(f"Peak VRAM (allocated): {torch.cuda.max_memory_allocated()/1024**2:.2f} MB")
        print(f"Peak VRAM (reserved): {torch.cuda.max_memory_reserved()/1024**2:.2f} MB")


        # Save the output
        image.save(output_path)
        print(f"Image saved to {output_path}")

    def thin_image(self, img):
        """
        Performs skeletonization (thinning) on a binary image.
        This uses an iterative morphological approach to reduce edges to 1-pixel thick lines.
        """
        _, binary = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)

        binary = (binary == 255).astype(np.uint8)

        skeleton = np.zeros_like(binary)
        done = False

        element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3,3))

        while not done:
            eroded = cv2.erode(binary, element)
            temp = cv2.dilate(eroded, element)
            temp = cv2.subtract(binary, temp)
            skeleton = cv2.bitwise_or(skeleton, temp)
            binary = eroded.copy()

            if cv2.countNonZero(binary) == 0:
                done = True

        skeleton = (skeleton * 255).astype(np.uint8)
        return skeleton

    def image_to_simplified_strokes(self, input_path, output_path, canny_thresh1=100, canny_thresh2=200):
        img = cv2.imread(input_path)
        if img is None:
            raise ValueError("Could not open or find the image.")
        
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        gray_filtered = cv2.bilateralFilter(gray, d=9, sigmaColor=75, sigmaSpace=75)
        
        edges = cv2.Canny(gray_filtered, canny_thresh1, canny_thresh2)
        
        thinned_edges = self.thin_image(edges)

        # new_w, new_h = 540, 360
        # final_img = np.zeros((new_h, new_w), dtype = img.dtype)

        # x_offset = (new_w - 360) // 2
        # y_offset = 0
        # final_img[y_offset:y_offset + 360, x_offset:x_offset+360] = thinned_edges
        

        
        # cv2.imwrite(output_path, final_img)
        cv2.imwrite(output_path, thinned_edges)



PATH_TEMP_FILE = './temp_files'
if __name__ == '__main__':
    text_to_image = TextToImage()
    output_img_path = PATH_TEMP_FILE + "/output.png"
    simplified_img_path = PATH_TEMP_FILE + "/simplified_strokes.png"
    text_path = PATH_TEMP_FILE + "/translated_text.txt"

    # If you want to get raw text from command line arguments:
    # raw_text = " ".join(sys.argv[1:])
    raw_text = "I want to draw a flowers."

    with open(text_path, "r") as f:
        raw_text = f.read()

    # Step 1: Extract illustration elements
    extracted_elements = text_to_image.extract_illustration_elements(raw_text)
    print("Extracted Elements:")
    print(extracted_elements)

    # Step 2: Get a suitable description for the T2I prompt
    objects_description = text_to_image.get_objects_description(extracted_elements)
    print("Objects Description:", objects_description)

    # Step 3: Generate the initial line-art style image
    # The output image will be saved as "output.png"
    text_to_image.generate_line_art_image(objects_description, output_path=output_img_path)

    # Step 4: Post-process to simplify strokes
    # The final simplified line-art image will be saved as "simplified_strokes.png"
    text_to_image.image_to_simplified_strokes(output_img_path, simplified_img_path)
    print("Final simplified line-art image saved as simplified_strokes.png")

