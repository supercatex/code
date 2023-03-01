from PIL import Image
from clip_interrogator import Config, Interrogator

# ci = Interrogator(Config(clip_model_name="ViT-L-14/openai"))
cfg = Config()
cfg.apply_low_vram_defaults()
cfg.device = "cuda"
ci = Interrogator(cfg)
print(ci.config.device)

while True:
    s = input()
    image = Image.open(r"%s" % s).convert('RGB')
    print(ci.generate_caption(image))
    # print(ci.interrogate(image))
