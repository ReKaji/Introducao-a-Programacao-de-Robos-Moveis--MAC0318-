## Introdução

Este repositório contém trabalhos desenvolvidos na disciplina "Introdução à Programação de Robôs Móveis (MAC0318)" — com foco em arquitetura de software aplicada a sistemas embarcados e controle de robôs. Os artefatos demonstram projeto e implementação de um sistema de simulação/controle para um táxi autônomo em plataforma LEGO NXT (leJOS). O conteúdo foi organizado para evidenciar decisões técnicas, algoritmos e competências práticas relevantes para engenharia de software.

---

## Projetos

- Projeto Final — Simulação de táxi autônomo
  - Local: `Projeto Final - Simulação de táxi autônomo/Taxi.java`
  - Linguagem: Java (leJOS NXJ API)
  - Descrição resumida: implementação de um controlador para táxi autônomo sobre plataforma LEGO NXT. O robô detecta cilindros (passageiros), identifica destino a partir de cor, realiza manobra de captura/entrega usando garra, e navega entre pontos pré-definidos usando navegação por waypoints e odometria.
  - Principais componentes implementados:
    - Loop de percepção-ação com múltiplos sensores: sensores de luz (linha), sensor ultrassônico (distância) e sensor de cor (identificação de destino).
    - Controle em malha fechada (PID) para seguimento de linha e retorno.
    - Planejamento simples por waypoints + uso de `Navigator` e `PoseProvider` para navegação baseada em odometria.
    - Ações de manipulação (garra) e sequenciamento de tarefas (pegar, transportar, devolver).

---

## Tecnologias e conceitos

- Java (programação orientada a objetos) — projeto estruturado em funções reutilizáveis e constantes configuráveis.
- leJOS NXJ API (biblioteca para LEGO Mindstorms NXT): `Navigator`, `DifferentialPilot`, `OdometryPoseProvider`, sensores e motores.
- Controle: PID discreto para manter o robô sobre faixas coloridas (ajuste de ganhos, anti-windup implícito via saturação de velocidade).
- Localização e Navegação: odometria, waypoint navigation e rotinas de trajetórias (followPath, goTo).
- Percepção: fusão simples entre leituras de cor, luz e ultrassom para decisões (detecção de cilindros, leitura de cor do destino, evita obstáculos).
- Engenharia de software: configuração centralizada (constantes), separação de responsabilidades (percepção, controle, navegação, manipulação), tratamento básico de exceções e logging via `System.out`.

---

## Como compilar e executar (notas e orientações)

Observação: o código depende da API leJOS e costuma ser compilado/empacotado usando o toolchain do leJOS NXJ ou adicionando os jars da API no classpath. Se for compilar localmente sem a toolchain, será necessário apontar o classpath para o(s) jar(s) do leJOS.

Exemplos (genéricos):

1) Compilação direta com `javac` (se você tiver os jars do leJOS disponíveis):

```bash
# Ajuste o caminho para o(s) jar(s) do leJOS
JARS="/path/to/lejos.jar:/path/to/lejos-nxj-core.jar"
javac -cp "$JARS" "Projeto Final - Simulação de táxi autônomo/Taxi.java"
```

2) Compilar e enviar para a NXT usando a toolchain leJOS (recomendado quando disponível):

```bash
# comandos dependem da versão do leJOS instalada (nxj-toolchain / ant / eclipse plugin)
# ex: nxj-javac Projeto\ Final\ -\ Simulação\ de\ táxi\ autônomo/Taxi.java
# ex: nxj-flash Taxi
```

3) Execução/local testing

- Testes unitários convencionais não estão incluídos; para validação local sem hardware, recomendamos extrair lógica não-hardware para classes testáveis e simular sensores.

---

## Observações técnicas / pontos de atenção

- O projeto foi desenvolvido para a plataforma NXT (leJOS). A interação direta com hardware (motores, sensores) significa que a lógica requer testes em bancada e ajustes de parâmetros (ganhos PID, distâncias). Muitos valores estão expostos como constantes para facilitar calibração.
- Há trade-offs explícitos: decisões simples de filtragem por cor, uso de odometria (sujeita a drift), e caminhos hard-coded por waypoints. Cada escolha foi feita para equilibrar robustez e simplicidade no contexto da disciplina.

---

## Resultados e validação

- Demonstração: execução onboard em um robô NXT real (seguimento de linha, detecção e entrega de 'passageiro').
- Métricas práticas a serem coletadas para produção: taxa de sucesso na captura/entrega, desvio médio da linha (mm), tempo médio de missão, taxa de falsos positivos na identificação por cor.

---
