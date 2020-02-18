
#pragma once

template <typename Type>
class CustomOscillator {
public:

    //Building the filter that will become our oscillator
    CustomOscillator() {
        auto& osc = processorChain.template get<oscIndex>();
        osc.initialise([](Type x) {
            return jmap(x, Type(-MathConstants<double>::pi), Type(MathConstants<double>::pi), Type(-2), Type(1));
            }, 4);
    }

    void setFrequency (Type newValue, bool force = false) {
        auto& osc = processorChain.template get<oscIndex>();
        osc.setFrequency(newValue, force);
    }

    void setLevel (Type newValue) {
        auto& gain = processorChain.template get<gainIndex>();
        gain.setGainLinear(newValue);
    }

    void reset() noexcept {
        processorChain.reset(); 
    }

    template <typename ProcessContext>
    void process (const ProcessContext& context) noexcept {
        processorChain.process(context);
    }

    void prepare (const juce::dsp::ProcessSpec& spec) {
        processorChain.prepare(spec);
    }

private:

    enum {
        oscIndex,
        gainIndex
    }; 

    //Setting up our oscillator to interact properly with the MIDI chain
    juce::dsp::ProcessorChain<juce::dsp::Oscillator<Type>, juce::dsp::Gain<Type>> processorChain;

};

class Voice  : public juce::MPESynthesiserVoice {
public:

    //Setting up a dual voice synth to allow for better readouts
    Voice() {
        auto& masterGain = processorChain.get<masterGainIndex>();
        masterGain.setGainLinear(0.7f);

        auto& filter = processorChain.get<filterIndex>();
        filter.setCutoffFrequencyHz(1000.0f);
        filter.setResonance(0.9f);
        lfo.initialise([](float x) { return std::sin(x); }, 128);
        lfo.setFrequency(2.0f);
    }

    void prepare (const juce::dsp::ProcessSpec& spec) {
        tempBlock = juce::dsp::AudioBlock<float> (heapBlock, spec.numChannels, spec.maximumBlockSize);
        processorChain.prepare (spec);
        lfo.prepare({ spec.sampleRate / lfoUpdateRate, spec.maximumBlockSize, spec.numChannels });
    }

    void noteStarted() override {
        auto velocity = getCurrentlyPlayingNote().noteOnVelocity.asUnsignedFloat();
        auto freqHz = (float)getCurrentlyPlayingNote().getFrequencyInHertz();

        processorChain.get<osc1Index>().setFrequency(freqHz, true);
        processorChain.get<osc1Index>().setLevel(velocity);

        processorChain.get<osc2Index>().setFrequency(freqHz * 1.01f, true);
        processorChain.get<osc2Index>().setLevel(velocity);
    }

    void notePitchbendChanged() override {
        auto freqHz = (float) getCurrentlyPlayingNote().getFrequencyInHertz();
        processorChain.get<osc1Index>().setFrequency (freqHz);
        processorChain.get<osc2Index>().setFrequency(freqHz * 1.01f);
    }

    void noteStopped (bool) override {
        clearCurrentNote();
    }

    void notePressureChanged() override {
    }
    void noteTimbreChanged()   override {
    }
    void noteKeyStateChanged() override {
    }


    //This is the main LFO Filter calculations
    void renderNextBlock(AudioBuffer<float>& outputBuffer, int startSample, int numSamples) override {
        auto output = tempBlock.getSubBlock(0, (size_t)numSamples);
        output.clear();

        for (size_t pos = 0; pos < numSamples;) {
            auto max = jmin(static_cast<size_t> (numSamples - pos), lfoUpdateCounter);
            auto block = output.getSubBlock(pos, max);

            juce::dsp::ProcessContextReplacing<float> context(block);
            processorChain.process(context);

            pos += max;
            lfoUpdateCounter -= max;

            if (lfoUpdateCounter == 0) {
                lfoUpdateCounter = lfoUpdateRate;
                auto lfoOut = lfo.processSample(0.1f);
                auto curoffFreqHz = jmap(lfoOut, -1.0f, 1.0f, 100.0f, 2000.0f);
                processorChain.get<filterIndex>().setCutoffFrequencyHz(curoffFreqHz);
            }
        }

        juce::dsp::AudioBlock<float>(outputBuffer)
            .getSubBlock((size_t)startSample, (size_t)numSamples)
            .add(tempBlock);
    }

private:

    juce::dsp::Oscillator<float> lfo;

    juce::HeapBlock<char> heapBlock;
    juce::dsp::AudioBlock<float> tempBlock;

    enum
    {
        osc1Index,
        osc2Index,
        filterIndex,
        masterGainIndex
    };

    juce::dsp::ProcessorChain<CustomOscillator<float>, CustomOscillator<float>,
        juce::dsp::LadderFilter<float>, juce::dsp::Gain<float>> processorChain;
    static constexpr size_t lfoUpdateRate = 101;
    size_t lfoUpdateCounter = lfoUpdateRate;
};

class AudioEngine  : public juce::MPESynthesiser {
public:
    static constexpr auto maxNumVoices = 4;

    AudioEngine() {
        for (auto i = 0; i < maxNumVoices; ++i)
            addVoice (new Voice);

        setVoiceStealingEnabled (true);
    }

    void prepare (const juce::dsp::ProcessSpec& spec) noexcept {
        setCurrentPlaybackSampleRate (spec.sampleRate);

        for (auto* v : voices)
            dynamic_cast<Voice*> (v)->prepare (spec);
    }

private:
    void renderNextSubBlock (AudioBuffer<float>& outputAudio, int startSample, int numSamples) override {
        MPESynthesiser::renderNextSubBlock (outputAudio, startSample, numSamples);
    }
};

template <typename SampleType>
class AudioBufferQueue {
public:
    static constexpr size_t order = 9;
    static constexpr size_t bufferSize = 1U << order;
    static constexpr size_t numBuffers = 5;

    void push (const SampleType* dataToPush, size_t numSamples) {
        jassert (numSamples <= bufferSize);

        int start1, size1, start2, size2;
        abstractFifo.prepareToWrite (1, start1, size1, start2, size2);

        jassert (size1 <= 1);
        jassert (size2 == 0);

        if (size1 > 0)
            FloatVectorOperations::copy (buffers[(size_t) start1].data(), dataToPush, (int) jmin (bufferSize, numSamples));

        abstractFifo.finishedWrite (size1);
    }

    void pop (SampleType* outputBuffer) {
        int start1, size1, start2, size2;
        abstractFifo.prepareToRead (1, start1, size1, start2, size2);

        jassert (size1 <= 1);
        jassert (size2 == 0);

        if (size1 > 0)
            FloatVectorOperations::copy (outputBuffer, buffers[(size_t) start1].data(), (int) bufferSize);

        abstractFifo.finishedRead (size1);
    }

private:
    AbstractFifo abstractFifo { numBuffers };
    std::array<std::array<SampleType, bufferSize>, numBuffers> buffers;
};

template <typename SampleType>
class ScopeDataCollector {
public:
    ScopeDataCollector (AudioBufferQueue<SampleType>& queueToUse)
        : audioBufferQueue (queueToUse) {
    }

    void process (const SampleType* data, size_t numSamples) {
        size_t index = 0;

        if (state == State::waitingForTrigger) {
            while (index++ < numSamples) {
                auto currentSample = *data++;

                if (currentSample >= triggerLevel && prevSample < triggerLevel) {
                    numCollected = 0;
                    state = State::collecting;
                    break;
                }

                prevSample = currentSample;
            }
        }

        if (state == State::collecting) {
            while (index++ < numSamples) {
                buffer[numCollected++] = *data++;

                if (numCollected == buffer.size()) {
                    audioBufferQueue.push (buffer.data(), buffer.size());
                    state = State::waitingForTrigger;
                    prevSample = SampleType (100);
                    break;
                }
            }
        }
    }

private:
    AudioBufferQueue<SampleType>& audioBufferQueue;
    std::array<SampleType, AudioBufferQueue<SampleType>::bufferSize> buffer;
    size_t numCollected;
    SampleType prevSample = SampleType (100);

    static constexpr auto triggerLevel = SampleType (0.05);

    enum class State { waitingForTrigger, collecting } state { State::waitingForTrigger };
};

template <typename SampleType>
class ScopeComponent  : public juce::Component,
                        private Timer 
{

public:
    using Queue = AudioBufferQueue<SampleType>;

    ScopeComponent (Queue& queueToUse)
        : audioBufferQueue (queueToUse) {
        sampleData.fill (SampleType (0));
        setFramesPerSecond (30);
    }

    void setFramesPerSecond (int framesPerSecond) {
        jassert (framesPerSecond > 0 && framesPerSecond < 1000);
        startTimerHz (framesPerSecond);
    }

    void paint (Graphics& g) override {
        g.fillAll (juce::Colours::black);
        g.setColour (juce::Colours::white);

        auto area = getLocalBounds();
        auto h = (SampleType) area.getHeight();
        auto w = (SampleType) area.getWidth();

        // Creating our false Oscilloscope
        auto scopeRect = Rectangle<SampleType> { SampleType (0), SampleType (0), w, h / 2 };
        plot (sampleData.data(), sampleData.size(), g, scopeRect, SampleType (1), h / 4);

        // Defining the spectrum range of our Oscilloscope
        auto spectrumRect = Rectangle<SampleType> { SampleType (0), h / 2, w, h / 2 };
        plot (spectrumData.data(), spectrumData.size() / 4, g, spectrumRect);
    }

    void resized() override {
    }

private:
    Queue& audioBufferQueue;
    std::array<SampleType, Queue::bufferSize> sampleData;

    juce::dsp::FFT fft { Queue::order };
    using WindowFun = juce::dsp::WindowingFunction<SampleType>;
    WindowFun windowFun { (size_t) fft.getSize(), WindowFun::hann };
    std::array<SampleType, 2 * Queue::bufferSize> spectrumData;

    void timerCallback() override {
        audioBufferQueue.pop (sampleData.data());
        FloatVectorOperations::copy (spectrumData.data(), sampleData.data(), (int) sampleData.size());

        auto fftSize = (size_t) fft.getSize();

        jassert (spectrumData.size() == 2 * fftSize);
        windowFun.multiplyWithWindowingTable (spectrumData.data(), fftSize);
        fft.performFrequencyOnlyForwardTransform (spectrumData.data());

        static constexpr auto mindB = SampleType (-160);
        static constexpr auto maxdB = SampleType (0);


        //Generating our gain upper and lower limits
        for (auto& s : spectrumData)
            s = jmap (jlimit (mindB, maxdB, juce::Decibels::gainToDecibels (s) - juce::Decibels::gainToDecibels (SampleType (fftSize))), mindB, maxdB, SampleType (0), SampleType (1));

        repaint();
    }

    static void plot (const SampleType* data,
                      size_t numSamples,
                      Graphics& g,
                      juce::Rectangle<SampleType> rect,
                      SampleType scaler = SampleType (1),
                      SampleType offset = SampleType (0))
    {
        auto w = rect.getWidth();
        auto h = rect.getHeight();
        auto right = rect.getRight();

        auto center = rect.getBottom() - offset;
        auto gain = h * scaler;

        for (size_t i = 1; i < numSamples; ++i)
            g.drawLine ({ jmap (SampleType (i - 1), SampleType (0), SampleType (numSamples - 1), SampleType (right - w), SampleType (right)),
                          center - gain * data[i - 1],
                          jmap (SampleType (i), SampleType (0), SampleType (numSamples - 1), SampleType (right - w), SampleType (right)),
                          center - gain * data[i] });
    }
};

class OscillatorFilterProcessor  : public AudioProcessor {
public:
    OscillatorFilterProcessor()
         : AudioProcessor (BusesProperties().withOutput ("Output", AudioChannelSet::stereo(), true)) {
    }


    //Preparing our DSP Engine for playback
    void prepareToPlay (double sampleRate, int samplesPerBlock) override {
        audioEngine.prepare ({ sampleRate, (uint32) samplesPerBlock, 2 });
        midiMessageCollector.reset (sampleRate);
    }

    void releaseResources() override {
    }

    bool isBusesLayoutSupported (const BusesLayout& layouts) const override {
        // This is the place where you check if the layout is supported.
        // In this application we only support mono or stereo, no surround sound.
        if (layouts.getMainOutputChannelSet() != AudioChannelSet::mono()
         && layouts.getMainOutputChannelSet() != AudioChannelSet::stereo())
            return false;

        return true;
    }

    void processBlock (AudioSampleBuffer& buffer, MidiBuffer& midiMessages) override {
        ScopedNoDenormals noDenormals;
        auto totalNumInputChannels  = getTotalNumInputChannels();
        auto totalNumOutputChannels = getTotalNumOutputChannels();

        midiMessageCollector.removeNextBlockOfMessages (midiMessages, buffer.getNumSamples());

        for (int i = totalNumInputChannels; i < totalNumOutputChannels; ++i)
            buffer.clear (i, 0, buffer.getNumSamples());

        audioEngine.renderNextBlock (buffer, midiMessages, 0, buffer.getNumSamples());
        scopeDataCollector.process (buffer.getReadPointer (0), (size_t) buffer.getNumSamples());
    }

    AudioProcessorEditor* createEditor() override {
        return new OscillatorFilterProcessorEditor (*this);
    }

    //Checking JUCE midi modules. Since this is a bit of a long section I'll mark the end.

    bool hasEditor() const override { 
        return true; 
    }

    const String getName() const override { 
        return JucePlugin_Name; 
    }

    bool acceptsMidi() const override { 
        return true; 
    }

    bool producesMidi() const override { 
        return false; 
    }
    
    bool isMidiEffect() const override { 
        return false; 
    }

    double getTailLengthSeconds() const override { 
        return 0.0; 
    }

    int getNumPrograms() override { 
        return 1; 
    }

    int getCurrentProgram() override { 
        return 0; 
    }

    void setCurrentProgram (int) override {
    }

    const String getProgramName (int) override { 
        return {
        }; 
    }

    void changeProgramName (int, const String&) override {
    }

    void getStateInformation (MemoryBlock&) override {
    }

    void setStateInformation (const void*, int) override {
    }

    MidiMessageCollector& getMidiMessageCollector() noexcept { 
        return midiMessageCollector; 
    }

    AudioBufferQueue<float>& getAudioBufferQueue() noexcept  { 
        return audioBufferQueue; 
    }
    //End of module gathering.

private:
    class OscillatorFilterProcessorEditor  : public AudioProcessorEditor {
    public:

        //UI Setup

        OscillatorFilterProcessorEditor (OscillatorFilterProcessor& p)
            : AudioProcessorEditor (&p),
              dspProcessor (p),
              scopeComponent (dspProcessor.getAudioBufferQueue()) {
            addAndMakeVisible (midiKeyboardComponent);
            addAndMakeVisible (scopeComponent);

            setSize (800, 900);

            auto area = getLocalBounds();
            scopeComponent.setTopLeftPosition (0, 80);
            scopeComponent.setSize (area.getWidth(), area.getHeight() - 100);

            midiKeyboardComponent.setMidiChannel (2);
            midiKeyboardState.addListener (&dspProcessor.getMidiMessageCollector());
        }

        ~OscillatorFilterProcessorEditor() override
        {
            midiKeyboardState.removeListener (&dspProcessor.getMidiMessageCollector());
        }

        void paint (Graphics& g) override
        {
            g.fillAll (getLookAndFeel().findColour (ResizableWindow::backgroundColourId));
        }

        void resized() override
        {
            auto area = getLocalBounds();
            midiKeyboardComponent.setBounds (area.removeFromTop (80).reduced (8));
        }

    private:
        OscillatorFilterProcessor& dspProcessor;
        MidiKeyboardState midiKeyboardState;
        MidiKeyboardComponent midiKeyboardComponent { midiKeyboardState, MidiKeyboardComponent::horizontalKeyboard };
        ScopeComponent<float> scopeComponent;

        JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (OscillatorFilterProcessorEditor)
    };

    AudioEngine audioEngine;
    MidiMessageCollector midiMessageCollector;
    AudioBufferQueue<float> audioBufferQueue;
    ScopeDataCollector<float> scopeDataCollector { audioBufferQueue };

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (OscillatorFilterProcessor)
};
