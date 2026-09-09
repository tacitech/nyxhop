# Licence

Every NyxHop board needs a licence. The first ten are free, commercial use included; past that,
and for the coming feature generations, write to us and we will quote you. Nothing expires,
nothing counts hours, and the board never contacts us.

## Tiers

| | free | anything beyond that |
|---|---|---|
| who it is for | anyone, hobby or commercial, up to ten boards | more boards, a board of your own, features of your own, or NyxHop inside something you sell |
| boards | 10 per email | as many as the job needs |
| feature generations | the current one, with all its bug fixes, for ever | agreed with the work |
| support | community (Discussions) | agreed with the work |
| price | 0 | write to tacitechvn@gmail.com |

**Free covers commercial use too.** Sell what you build with it, run it in a company, put it in
a product. Ten boards per email address, and nothing in them expires.

**Licences are per board, never per pair.** A board is a board: which end it plays is a flag
when you flash it. A ground station serving three aircraft is four licences.

A *feature generation* is a release that adds capabilities (for example HD video, encryption).
Bug-fix releases belong to the generation they fix and load on every licence of that generation.

## Getting a licence

1. In the app, open **Licence** and press **Copy** next to the DNA (15 hex digits). The aircraft's
   DNA shows in the ground app while the link is up, or in `nyx-tx` on the aircraft computer.
2. Request it at **[nyxhop.com/licence.html](https://nyxhop.com/licence.html)**: a free licence comes
   back on the page at once. Paid licences: write to **tacitechvn@gmail.com** with the DNA and the
   order number, and the file comes back by email.
3. You receive a text file `nyxhop-<dna>.lic`.
4. Paste its contents into the app's licence box and press **Apply here**. For the aircraft, paste
   its file into the ground app and press **Send to aircraft**: the licence goes over the control
   link and is accepted within a second.

The licence pill turns to *licensed*. The file is stored on the board (and on the E200's SD card)
and loads at every boot.

## Grace period

A board without a licence works for 20 hours of operation, counted by the board, so you can test
before asking for a licence. After that video stops until a licence is loaded; control, pairing
and the apps keep working so you can still read the DNA and apply the file.

## What the DNA is, and privacy

The DNA is the factory serial number of the FPGA chip on the board. It identifies the chip and
nothing else: no location, no network, no personal data. A licence is bound to one DNA and works
on that chip only; someone who knows your DNA cannot use it for anything.

We keep your email in our records to count free licences. The licence file itself carries the
DNA, the licence data and an optional name; **it does not contain your email**, so a board can be
resold without exposing the previous owner.

## Console

On a board console (`nc <board> 7202`):

```
license                 state, DNA, generation, hours
license put <hex>       load a licence (hex of the file text)
license file <path>     load a licence from a file on the board
license far <hex>       ground end: push a licence to the aircraft over the air
license clear           remove the licence file (takes effect at the next power cycle)
```

`hop status` includes the `lic_*` lines.
