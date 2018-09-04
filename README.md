Testado no windows 64 bits com o simpleTest

Adicionados libs para MAC e Linux

Porta setada para remote API: 19990

Para alterar a porta deve-se modificar o arquivo remoteApiConnections.txt na raíz da pasta de instalação do V-REP

Windows
- Python 3.7

Whl retirados de https://www.lfd.uci.edu/~gohlke/pythonlibs/
- Numpy 1.15
  pip install "numpy-1.15.1+mkl-cp37-cp37m-win_amd64.whl"

- OpenCV 3.4.3
  pip install "opencv_python-3.4.3-cp37-cp37m-win_amd64.whl"
