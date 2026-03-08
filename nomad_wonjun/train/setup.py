# pip install -e train 폴더 할 때 필요. 이를 통해 site-packages 폴더에 링크 형태로(-e 모드여서)(내가 바꾸면 pip에 있는 것도 연동돼서 알아서 바뀌는 개념) 다운로드됨 
# 아래 find_packages() 할 때 __init__.py가 없는 폴더는 패키지로 인식하지 않아서, 결국 vint_train 폴더만 다운로드됨
# 그래서 애들 import하는 코드 보면 vint_train부터 해서 import하는 것
# deployment에 있는 파일 등 다양한 곳에서 import하기 때문에 경로 무관하도록 pip해놓고 사용하는 것
from setuptools import setup, find_packages

setup(
    name="vint_train",
    version="0.1.0",
    packages=find_packages(),
)
