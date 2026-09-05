# analog

Analog modulation and receiver signal conditioning: reading an angle-modulated carrier
back to its message, and holding a stream's level and envelope where the rest of a chain
expects them. The blocks here work on a sample stream; nothing above a sample — a symbol,
a frame — is in scope.

The blocks stay in `gr::blocks::analog` and ship no compatibility import: the module is new, so there is no
earlier `gr::analog` for one to be compatible with.
