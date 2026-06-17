import numpy as np
import pandas as pd
from tkinter import ttk
import tkinter as tk

# 1. 파일 로드 (예시로 arr_0 행렬 가져오기)
data = np.load('IMU\verification_tool\output\collected_data_100s.npz')
matrix = data[data.files[0]]  

# 2. 넘파이 행렬을 엑셀 같은 데이터프레임으로 변환
df = pd.DataFrame(matrix)

# 3. 화면에 행렬 형태로 팝업창 띄우기
root = tk.Tk()
root.title("행렬 뷰어")
txt = tk.Text(root)
txt.insert(tk.END, df.to_string()) # 행렬을 문자열 표 형태로 찍어줌
txt.pack()
root.mainloop()