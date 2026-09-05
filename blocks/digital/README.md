# digital

Framing and coding: what a digital chain does with symbols once a synchronizer has
produced them, and what a modulator does with bits before one is needed. The
constellation itself is a value type in the `gr::digital` algorithm layer; the blocks
here map, code and frame around it.

`gr::digital` is the algorithm layer's namespace, so the blocks here stay in
`gr::blocks::digital` and ship no compatibility import, which would collide with it.
