# adsb

`PpmFramer` reads the 1090 MHz Mode S downlink out of a magnitude stream: it scans for the four-pulse
preamble, slices the pulse-position chips that follow it, and publishes a candidate frame's octets as
a `gr::DataSet<std::uint8_t>` record with the parity result and the timing beside them. The message
layer that says what an admitted frame _says_ follows in this module.

The framer is here rather than in `digital` because pulse-position modulation at this preamble, frame
length and parity is Mode S and nothing else. The pulse-position scanner it runs on is the general
kernel and stays in the algorithm layer, as `gr::digital::PpmScanner`.

The blocks stay in `gr::blocks::adsb` and ship no compatibility import: the module is new, so there is
no earlier `gr::adsb` for one to be compatible with.
