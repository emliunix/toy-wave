# Toy Wave

The ultimate goal is to construct a usable speech interface for LLM.

On the LLM side, there're several multimodal models out there, but the inference engines lack matual support for them. Especially a stateful API that keeps the tokenized speech at server side to avoid repeated processing.

On the device side, a voice processing pipeline is required to capture speechs.

## Why a speech interface

Speech is one of the most natural ways of human communication. It's append only, like typing without a backspace, and can happen at anytime. One of the difficulties interacting with machines, LLMs is that thoughtful typing still takes a lot of engergy, and hence the bandwith of information exchange between you and the machine is limited. I believe a speech interface can drastically improve the bandwith. And hence transfer a lot of context for the machine to comprehend and assist.

A speech input, visual output, isn't it great.

## Progress

- [x] a toy USB interface mic
- [ ] a toy audio processing program
  - [ ] let's start with a fft freq spectrogram view

### USB MIC

It's a hello world project, but should pull fundamental embedded voice processing things together.

I'm using a **xiao ble sense** board with a nrf52840 chip and a onboard PDM MIC. 

Here're notes of problems encountered:

There're 2 issues with the upstream xiao ble board support in zephyr project (Nordic's NCS SDK is based on zephyr). 

1. The chip is pre-flashed with a bootloader but the board config doesn't offset the firmware address on flash, so your program won't run. It took a lot of time debugging cause the feedback is empty. Tried SWD debugger but it stops at a random place and you have nowhere proper to set a breakpoint. The final technique is **guess**. Though I end up wipped out the bootloader but it the bootloader should still be useful if you add the flash base offset by yourself. The device tree code parition is not enough.
2. The PDM mic is not working out of the box. Because it's not powered on by default. And yes, the difficulty is you don't have any feedback, the data read back is a flat constant. The mis is onboard you don't even have pins to test. And by **guess**, enabled power in the device tree overlay, and it works.
3. The USB stack and audio data processing is a little bit harder to handle than expected. It's basically an async stream pipe but you don't have the stream abstraction in place. The API is callback based. And it's also special for audio. You would prefer drop old data if you can't catch up.
  a. I was impressed there's a sophisticated audio pipeline API in web browser. It's a circular buffer with a audio source and a set of processing nodes attached. Maybe I can follow the same idea.
