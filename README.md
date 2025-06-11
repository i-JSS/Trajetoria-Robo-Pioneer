# Trajetória Robo Pioneer

|             Aluno             | Matrícula |
|:-----------------------------:|:---------:|
| João Antonio Ginuino carvalho | 221008150 |


---

## Sobre

Controle de trajetória do robo Pioneer para atingir multiplas bolas, fazendo o melhor caminho entre as bolas utilizando uma arvore geradora mínima e controlando a movimentação do robo por meio da cinematica direta.

> O trabalho irá plotar a trajetória que o robo realizará em seguida irá movimentar o mesmo com o controle PID.

Plot da trajetória a um ponto fixo:

![trajetória de multiplos](src/images/pontofixo.png)

Plot da trajetória a um ponto variável, movimentando o ponto no decorrer do tempo:

![trajetória de multiplos](src/images/pontovariavel.png)

Plot da trajetória de multiplos pontos:

![trajetória de multiplos](src/images/pontosmultiplos.png)

---

## Instalação

Linguagem: Python - 3.12

Biblioteca: numpy e matplotlib.pyplot

---

## Uso

Para única bola - abrir o arquivo **pioneer_one_ball.ttt** no coppeliaSim e executar o script **main.py**

Para multiplas bolas - abrir o arquivo **pioneer_multiples_balls.ttt** no coppeliaSim e executar o script **main.py**