
#include <JuceHeader.h>
#include "OscillatorFilter.h"

AudioProcessor* JUCE_CALLTYPE createPluginFilter() {
    return new OscillatorFilterProcessor();
}
