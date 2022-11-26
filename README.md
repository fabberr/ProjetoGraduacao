# Reconstrução 3D de Cenas e Objetos a Partir de Imagens Digitais
Aplicação CLI desenvolvida em C++ como Projeto de Graduação para o curso de Ciência da Computação da FIPP, Presidente Prudente - SP, 2021 - 2022.

Este projeto busca, a partir de uma coleção de imagens digitais, gerar uma malha tridimensional texturizada da cena observada nas fotos utilizando diversas técnicas de *Structure From Motion* (SfM) e *Multi-View Stereo* (MVS) encadeadas em um *pipeline* de execução.

#### Autores
> Fabrício Milanez (orientando)<br>
> Francisco Assis da Silva (orientador)<br>
> Flávio Pandur Albuquerque Cabral (orientador)

## Build
Atualmente, builds nativas são suportadas oficialmente somente em plataformas GNU/Linux devido à falta de suporte de algumas dependências do OpenCV contrib no Windows. Apesar da falta de suporte, esse projeto pode ser compilado no Windows usando o Subsistema do Windows Para Linux ([WSL2](https://learn.microsoft.com/windows/wsl/install)). Builds nativas em plataformas macOS não foram testadas porém devem funcionar.

Scripts para automatização de build estão planejados, mas por enquanto, para rodar o projeto é necessário fazer a instalação de todas as dependências listadas abaixo *manualmente*<!-- boa sorte kkkkkk -->.

### Dependências
- Git ([latest](https://git-scm.com/downloads) recomendado)
- Compilador C++ com [suporte ao C++20](https://en.cppreference.com/w/cpp/compiler_support#C.2B.2B20_features) (GCC 11+ recomendado)
  <br>**Nota**: Este projeto foi testado com GCC 11.3.0, outros compiladores como LLVM Clang não foram testados.
- Build System para C/C++ compatível com Linux:
  - GNU Make ou Ninja Build
- CMake ^3.21.4 ([latest](https://github.com/Kitware/CMake/releases/latest) recomendado)
- OpenCV [3.4.18](https://github.com/opencv/opencv/releases/tag/3.4.18):
  <br>**ATENÇÃO**: Todos o módulos do `opencv` e `opencv_contrib` utilizados devem estar na **mesma versão** (3.4.18).
  - `core`
  - `imgproc`
  - `imgcodecs`
  - `calib3d`
  - `viz`
  - `features2d`
  - `xfeatures2d` ([contrib](https://github.com/opencv/opencv_contrib/releases/tag/3.4.18))
  - `sfm` ([contrib](https://github.com/opencv/opencv_contrib/releases/tag/3.4.18))
- OpenMVS [^2.0](https://github.com/cdcseacave/openMVS/releases)
- Point Cloud Library (PCL) [^1.12.1](https://pointclouds.org/downloads/)
- Boost [1.80](https://www.boost.org/users/history/version_1_80_0.html):
  - `boost_filesystem`
  - `boost_graph`
  - `boost_iostreams`
  - `boost_program_options`
  - `boost_system`
  - `boost_serialization`

---
### To-do
- [x] Atualizar README.md
  - [x] Introdução
  - [x] Build
    - [x] Dependências
      - [x] verificar se falta alguma dep
      - [x] versões do Boost, OpenMVS e PCL usadas
- [ ] Adicionar licença (incluindo licenças de terceiros)
- [ ] Adicionar link para o artigo uma vez que ele for publicado
- [ ] Tornar o projeto cross-platform:
  - [ ] Criar script (Python ou Ruby) para instalar todas as dependências do projeto
  - [ ] Converter scripts de teste do projeto de Bash para outra linguagem cross-platform
  - [ ] Criar uma imagem Docker que rode o script de instalação acima
- [ ] Refatorar/limpar código-fonte (branch nova, manter fonte atual como está para que os resultados obtidos no artigo sejam reproduzíveis)
