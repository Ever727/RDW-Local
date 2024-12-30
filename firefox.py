from selenium import webdriver
from selenium.webdriver.common.by import By
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions as EC
import time
import sys

# 启动 Firefox 浏览器
driver = webdriver.Firefox()
json_path = sys.argv[1]

try:
    # 1. 访问网页
    print("开始访问网页")
    driver.get("http://0.0.0.0:8000")
    print("成功访问网页")

    # 2. 等待并点击 Input Config 按钮
    input_config_button = WebDriverWait(driver, 10).until(
        EC.element_to_be_clickable((By.XPATH, "//button[text()='Input Config']"))
    )
    input_config_button.click()
    print("成功点击 Input Config 按钮")

    # 3. 上传文件 test.json
    file_input = WebDriverWait(driver, 10).until(
        EC.presence_of_element_located((By.XPATH, "//input[@type='file']"))
    )
    file_input.send_keys("~/VR/RDW-Local/" + json_path)  # 替换为 test.json 的绝对路径
    # print("成功上传 test.json 文件")

    # 4. 点击关闭上传页面按钮
    start_button = WebDriverWait(driver, 10).until(
        EC.element_to_be_clickable((By.XPATH, "//*[@id='closeModalButton']"))
    )
    start_button.click()
    # print("成功点击 关闭 按钮")
    
    # 5. 点击Local按钮
    start_button = WebDriverWait(driver, 10).until(
        EC.element_to_be_clickable((By.XPATH, "//*[@id='localButton']"))
    )
    start_button.click()
    # print("成功点击 Local 按钮")

    # 6. 点击开始按钮
    start_button = WebDriverWait(driver, 10).until(
        EC.element_to_be_clickable((By.XPATH, "//button[text()='Simulate']"))
    )
    start_button.click()
    print("成功点击 simulate 按钮")

except Exception as e:
    print(f"发生错误: {e}")

finally:
    while True:
        time.sleep(1)
    driver.quit()
    print("关闭浏览器")
