from ollama import Client
from pydantic import BaseModel


class OutputFormat(BaseModel):
    name_of_person: str
    favorite_drink: str
    hair_color: str
    hair_style: str
    wear_glasses: bool
    look_at_camera: bool


class OllamaManager(object):
    def __init__(self, host="http://localhost:11434", system_prompt=None):
        self.client = Client(host=host)
        self.messages = []
        self.model = "gemma4:e2b"
        if system_prompt is not None:
            self.messages.append({
                "role": "system",
                "coontent": system_prompt
            })

    def send_message(self,
                     content: str,
                     name: str = None,
                     stream: bool = False,
                     format=None,
                     options=None,
                     images=None
    ):
        if name is not None:
            content = name + ": " + content

        message = {
            "role": "user",
            "content": content,
            "images": images
        }
        self.messages.append(message)

        response = self.client.chat(
            model=self.model,
            messages=self.messages,
            tools=[],
            think=True,
            stream=stream,
            format=format,
            options=options
        )

        thinking, content, tools = "", "", []
        if stream:
            in_thinking = True
            for chunk in response:
                if chunk.message.thinking:
                    thinking += chunk.message.thinking
                    # print(chunk.message.thinking, end="")
                elif in_thinking:
                    # print("\n")
                    in_thinking = False
                if chunk.message.content:
                    content += chunk.message.content
                    print(chunk.message.content, end="")
                if chunk.message.tool_calls:
                    tools.extend(chunk.message.tool_calls)
            print()
        else:
            thinking = response.message.thinking
            content = response.message.content
            if response.message.tool_calls:
                tools.extend(response.message.tool_calls)

        message = {
            "role": "assistant",
            "thinking": thinking,
            "content": content,
            "tool_calls": tools
        }
        self.messages.append(message)
        return message


if __name__ == "__main__":
    ai = OllamaManager(
        "http://172.22.11.33:11434",
        "你是服務型機器人助手"
    )
    message = ai.send_message(
        "圖片中的人說：「我的名字是健達，我喜歡飲茶。」",
        stream=False,
        format=OutputFormat.model_json_schema(),
        images=["abc.png"]
    )
    data = OutputFormat.model_validate_json(message["content"])
    print(data)
