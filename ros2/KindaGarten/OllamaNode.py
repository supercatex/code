from KindaGarten.basic import *


class OllamaNode(object):
    def __init__(self, host="http://127.0.0.1:11434", model_name="qwen3-vl:8b"):
        self.host = host 
        self.model_name = model_name
        self.client = Client(host=host)

    def send_message(self, text, img=None, img_path=None):
        t1 = time.time()

        messages = [{
            "role": "user",
            "content": text
        }]

        if img_path is not None:
            img = cv2.imread(img_path)

        if img is not None:
            b64_img = base64.b64encode(cv2.imencode(".jpg", img)[1]).decode("UTF-8")
            messages[0]["images"] = [b64_img]

        response = self.client.chat(self.model_name, messages=messages, think="low", stream=False)
        print("Response from ollama.", time.time() - t1)
        return response.message.content
    
