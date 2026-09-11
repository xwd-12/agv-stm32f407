"""
把 .mmd 文件编码成 mermaid.ink 链接，免费出 PNG。
用法：python 生成Mermaid链接.py
打开生成的链接，浏览器直接显示 PNG，右键另存即可。
"""
import zlib, base64, os

files = [
    "图1_系统整体框图.mmd",
    "图2_嵌套PID控制链路.mmd",
    "图3_AI自动扫描Pipeline.mmd",
    "图4_状态机调度总览.mmd",
    "图5_硬件连接拓扑.mmd",
]

for f in files:
    path = os.path.join(os.path.dirname(__file__) or ".", f)
    if not os.path.exists(path):
        print(f"⚠ 文件不存在: {f}")
        continue
    code = open(path, encoding="utf-8").read().strip()
    encoded = base64.urlsafe_b64encode(
        zlib.compress(code.encode("utf-8"), 9)
    ).decode("utf-8")
    url = f"https://mermaid.ink/img/{encoded}"
    print(f"=== {f} ===")
    print(url)
    print()
