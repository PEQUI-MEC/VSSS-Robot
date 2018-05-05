# VSSS-MBED
Código do Robô do VSSS para o mbed NXP LPC1768

## Dependências

A única toolchain suportada é a versão mais recente do GNU Arm Embedded.
Para instalá-la em distribuições baseadas no Ubuntu execute os comandos:
```
sudo add-apt-repository ppa:team-gcc-arm-embedded/ppa
sudo apt-get update
sudo apt-get install gcc-arm-embedded
```

A compilação também utiliza make e cmake:
```
sudo apt install make cmake
```
## Compilação

Código pode ser compilado utilizando os comandos:
```
mkdir build/
cd build/
cmake ..
make
```

ou através do script build.sh fornecido pelo projeto:

```
./build.sh
```
O resultado da compilação é um arquivo chamado VSSS.bin, que deve ser copiado para a memória do microcontrolador.
