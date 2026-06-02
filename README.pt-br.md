# Estimativa de Grandeza, Velocidade e Aceleração com Filtro de Kalman

*Leia em outros idiomas: [English](README.md)*

Este documento detalha o funcionamento conceitual e a implementação prática de **Filtros de Kalman Lineares de 2ª Ordem (2D)** e **3ª Ordem (3D)** para estimar uma grandeza ($\theta$), velocidade ($\omega$) e aceleração ($\alpha$) a partir das leituras brutas de um sensor.

---

## 1. Funcionamento Conceitual

### A. Filtro de Kalman 2D (Grandeza e Velocidade)
O estado do sistema é representado por um vetor de estados bidimensional:
$$\mathbf{x} = \begin{bmatrix} \theta \\ \omega \end{bmatrix} \begin{matrix} \text{ (Grandeza sendo medida)} \\ \text{ (Velocidade de variação da grandeza)} \end{matrix}$$

*   **Predição (Modelo Cinemático 2D):**
    A cada período de amostragem $\Delta t$ (ex: 1 ms), prevemos o próximo estado baseado no movimento linear uniforme:
    $$\theta_{k|k-1} = \theta_{k-1} + \omega_{k-1} \cdot \Delta t$$
    $$\omega_{k|k-1} = \omega_{k-1}$$
    
    A matriz de transição de estados $\mathbf{F}$ é:
    $$\mathbf{F} = \begin{bmatrix} 1 & \Delta t \\ 0 & 1 \end{bmatrix}$$

*   **Matriz de Medição $\mathbf{H}$:**
    $$\mathbf{H} = \begin{bmatrix} 1 & 0 \end{bmatrix}$$

---

### B. Filtro de Kalman 3D (Grandeza, Velocidade e Aceleração)
Para modelar sistemas com dinâmicas mais agressivas ou para obter uma estimativa direta da aceleração instantânea, expandimos o vetor de estados para 3 dimensões:
$$\mathbf{x} = \begin{bmatrix} \theta \\ \omega \\ \alpha \end{bmatrix} \begin{matrix} \text{ (Grandeza medida)} \\ \text{ (Velocidade de variação da grandeza)} \\ \text{ (Aceleração de variação da grandeza)} \end{matrix}$$

*   **Predição (Modelo Cinemático 3D):**
    Utilizando a equação do movimento uniformemente variado:
    $$\theta_{k|k-1} = \theta_{k-1} + \omega_{k-1} \cdot \Delta t + \frac{1}{2} \alpha_{k-1} \cdot \Delta t^2$$
    $$\omega_{k|k-1} = \omega_{k-1} + \alpha_{k-1} \cdot \Delta t$$
    $$\alpha_{k|k-1} = \alpha_{k-1}$$
    
    A matriz de transição de estados $\mathbf{F}$ é:
    $$\mathbf{F} = \begin{bmatrix} 1 & \Delta t & \frac{1}{2}\Delta t^2 \\ 0 & 1 & \Delta t \\ 0 & 0 & 1 \end{bmatrix}$$

*   **Matriz de Medição $\mathbf{H}$:**
    Como apenas a grandeza $\theta$ é medida fisicamente pelo sensor:
    $$\mathbf{H} = \begin{bmatrix} 1 & 0 & 0 \end{bmatrix}$$

*   **Equações da Covariância de Predição ($\mathbf{P}_{k|k-1} = \mathbf{F} \mathbf{P}_{k-1} \mathbf{F}^T + \mathbf{Q}$):**
    Para evitar aritmética de matrizes em tempo de execução no microcontrolador, as equações foram expandidas simbolicamente.
    Sendo $h = \frac{1}{2}\Delta t^2$:
    *   $a_0 = P_{00} + P_{10}\Delta t + P_{20}h$
    *   $a_1 = P_{01} + P_{11}\Delta t + P_{21}h$
    *   $a_2 = P_{02} + P_{12}\Delta t + P_{22}h$
    *   $a_4 = P_{11} + P_{21}\Delta t$
    *   $a_5 = P_{12} + P_{22}\Delta t$
    
    A matriz de covariância predita resultante é:
    *   $P_{00} = a_0 + a_1\Delta t + a_2 h + q_\theta$
    *   $P_{01} = a_1 + a_2\Delta t$
    *   $P_{02} = a_2$
    *   $P_{10} = P_{01}$
    *   $P_{11} = a_4 + a_5\Delta t + q_\omega$
    *   $P_{12} = a_5$
    *   $P_{20} = P_{02}$
    *   $P_{21} = P_{12}$
    *   $P_{22} = P_{22} + q_\alpha$

---

## 2. Parâmetros do Filtro e Ruídos

### Ruído de Medição ($R$)
O parâmetro $R$ define a variância do ruído elétrico/magnético de leitura do sensor.
*   **Cálculo pelo Datasheet:** Por exemplo, no caso do sensor AS5600, o ruído RMS (1-Sigma) é de $0.06^\circ$. Assim, a variância ideal correspondente é:
    $$R = (0.06^\circ)^2 = 0.0036\text{ (graus)}^2$$
*   **Ajuste Prático:** Para lidar com folgas mecânicas, vibrações e imperfeições do sensor no mundo real, o filtro (quando sintonizado para o AS5600) foi configurado com:
    $$R \approx 0.018\text{ (graus)}^2$$

### Ruído de Processo ($\mathbf{Q}$)
A matriz $\mathbf{Q}$ representa as incertezas do modelo físico.
*   **Sintonia 2D:**
    *   $q_\theta = 0.001$ (Incerteza muito baixa na estimação da grandeza)
    *   $q_\omega = 50.0$ (Permite acompanhar acelerações rápidas do sistema)
*   **Sintonia 3D:**
    *   $q_\theta = 0.001$ (Incerteza muito baixa na estimação da grandeza)
    *   $q_\omega = 10.0$ (Prioriza a filtragem de ruído na velocidade)
    *   $q_\alpha = 100.0$ (Permite resposta rápida às variações de aceleração)

---

## 3. Implementação das Estruturas (`src/kalman.h`)

As estruturas do filtro de Kalman 2D e 3D estão declaradas no cabeçalho do projeto:

```c
struct kalman_2d {
  float x[2];    /* Vetor de Estado: [0] = grandeza, [1] = velocidade */
  float P[2][2]; /* Matriz de covariância de erro */
  float Q_theta; /* Ruído de processo da grandeza */
  float Q_omega; /* Ruído de processo da velocidade */
  float R;       /* Ruído de medição */
};

struct kalman_3d {
  float x[3];    /* Vetor de Estado: [0] = grandeza, [1] = velocidade, [2] = aceleração */
  float P[3][3]; /* Matriz de covariância de erro */
  float Q_theta; /* Ruído de processo da grandeza */
  float Q_omega; /* Ruído de processo da velocidade */
  float Q_alpha; /* Ruído de processo da aceleração */
  float R;       /* Ruído de medição */
};
```
---
![SmartSensing.me Logo](https://smartsensing.me/ssme-logo.png)

## 📝 Descrição

Este projeto faz parte do ecossistema **SmartSensing.me** e vai além dos exemplos básicos encontrados na internet. Aqui, aplicamos os fundamentos reais da engenharia de instrumentação e sistemas embarcados de alta performance.

Diferente de conteúdos superficiais voltados apenas para cliques, este repositório entrega:
- **Ineditismo:** Implementações originais baseadas em quase 30 anos de experiência acadêmica.
- **Densidade Técnica:** Uso profissional do framework ESP-IDF e FreeRTOS.
- **Didática:** Código documentado e estruturado para quem busca evolução técnica real.

> "Transformamos sinais do mundo físico em inteligência digital, sem atalhos."

---

## 🛠️ Tecnologias e Compatibilidade
- **Linguagem:** C puro (C99 ou superior) e C++
- **Hardware Alvo:** Qualquer microcontrolador (ESP32, STM32, ARM Cortex, RISC-V, AVR, etc.) ou arquitetura desktop
- **Ambientes/RTOS:** ESP-IDF (como Componente nativo), Zephyr RTOS, FreeRTOS, Bare-metal, Desktop (Windows, Linux, macOS)
- **Build System:** CMake nativo
- **Simulação:** LTSpice (Modelagem e validação de sensores)

---

## 👤 Sobre o Autor

**José Alexandre de França** *Professor Adjunto no Departamento de Engenharia Elétrica da UEL*

Engenheiro Eletricista com quase três décadas de experiência no ensino de graduação e pós-graduação. Doutor em Engenharia Elétrica, pesquisador em instrumentação eletrônica e desenvolvedor de sistemas embarcados. O SmartSensing.me é o meu compromisso de elevar o nível da educação tecnológica no Brasil.

- 🌐 **Website:** [smartsensing.me](https://smartsensing.me)
- 📧 **E-mail:** [info@smartsensing.me](mailto:info@smartsensing.me)
- 📺 **YouTube:** [@smartsensingme](https://youtube.com/@smartsensingme)
- 📸 **Instagram:** [@smartsensing.me](https://instagram.com/smartsensing.me)

---

## 📄 Licença

Este projeto está sob a licença MIT. Veja o arquivo [LICENSE](LICENSE) para detalhes.

