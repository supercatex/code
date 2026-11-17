from ollama import chat
import pyautogui
import cv2
import numpy as np
import base64
import time

# https://gallery.selfboot.cn/en/games/sokoban?level=1

print("TOP-LEFT: ", end="")
time.sleep(5)
x1, y1 = pyautogui.position()
print(x1, y1)

print("BOTTOM-DOWN:", end="")
time.sleep(5)
x2, y2 = pyautogui.position()
print(x2, y2)


def move(action: str):
    """This function can move the player UP/DOWN/LEFT/RIGHT
    action: "up", "down", "left", "right" only four options.
    """
    action = action.strip()
    pyautogui.keyDown(action)
    time.sleep(0.1)
    pyautogui.keyUp(action)
    
messages = []

while True:
    time.sleep(1)
    img = pyautogui.screenshot(region=(x1, y1, (x2 - x1), (y2 - y1)))
    img = np.array(img)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    success, encoded_img = cv2.imencode(".jpg", img)
    data = base64.b64encode(encoded_img.tobytes()).decode()
    
    messages.append({
        "role": "user",
        "content": "You are the human. Your goal is push boxes to the red points. stone is wall. 7 rows, 6 cols. please call the tools to play this game.",
        "images": [data]
    })
    
    response = chat(
        model="bonsai-2:27b",
        messages=messages,
        tools=[move], 
        think=True,
        stream=True
    )
    
    in_thinking = False
    thinking, content = "", ""
    tool_calls = []
    for chunk in response:
        if chunk.message.thinking:
            if not in_thinking:
                in_thinking = True
                print("\033[32mThinking:\n\033[0m", end="", flush=True)
            print(f"\033[32m{chunk.message.thinking}\033[0m", end="", flush=True)
            thinking += chunk.message.thinking
        elif chunk.message.content:
            if in_thinking:
                in_thinking = False
                print("\n\nAnswer:\n", end="", flush=True)
            print(chunk.message.content, end="", flush=True)
            content += chunk.message.content
        if chunk.message.tool_calls:
            tool_calls.extend(chunk.message.tool_calls)
    print()
    
    messages.append({
        "role": "assistant",
        "thinking": thinking,
        "content": content,
        "tool_calls": tool_calls
    })

    if tool_calls:
        for tc in tool_calls:
            if tc.function.name == "move":
                move(**tc.function.arguments)
                print("%s('%s')" % (tc.function.name, tc.function.arguments["action"]))
    
                messages.append({
                    "role": "tool", 
                    "tool_name": tc.function.name, 
                    "content": str(tc.function.arguments["action"])
                })
    else: break
