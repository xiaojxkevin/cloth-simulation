from PIL import Image
import os

# Path to the directory containing JPEG images
image_dir = "./Coding/imgs"
output_gif = "./output.gif"

# Get a list of all JPEG files in the directory
images = [os.path.join(image_dir, f) for f in os.listdir(image_dir) if f.endswith(".jpeg")]
images.sort()  # Sort to ensure correct sequence

# Open images
frames = [Image.open(img) for img in images]

# Save the images as a GIF
frames[0].save(
    output_gif,
    save_all=True,
    append_images=frames[1:],
    optimize=True,
    duration=20,  # Duration between frames in milliseconds
    loop=0  # 0 for infinite loop
)

print(f"GIF saved as {output_gif}")
