# Controle de Nível e Vazão - UFES 2026
**Disciplina:** Microcontroladores / Sistemas Embarcados
**Autores:** André Nemer e Equipe

## 📌 Descrição
Sistema de controle em malha fechada utilizando **ATmega328P (Bare Metal)** para processamento de controle PID e **ESP32** como gateway IoT para monitoramento via **Blynk**.

## 🛠️ Arquitetura do Sistema
O projeto utiliza uma arquitetura distribuída:
* **ATmega328P:** Responsável pela leitura do sensor ultrassônico (Nível) e sensor de vazão, processamento do algoritmo PID e atuação no Servomotor e Bomba d'água.
* **ESP32:** Atua como ponte de comunicação (Gateway) entre o hardware e a nuvem via Wi-Fi.

## 🔧 Requisitos Atendidos
1. Linguagem C Bare Metal (Registradores).
2. Algoritmo PID Completo (P+I+D) com Anti-Windup.
3. Sensor Ultrassônico (Nível) e Sensor de Vazão.
4. Controle de Carga Indutiva (Bomba via MOSFET com Diodo Flyback).
5. Interface LCD 16x2 e Ajustes via Encoder Rotativo.
6. Telemetria e Monitoramento em Nuvem (Blynk).
