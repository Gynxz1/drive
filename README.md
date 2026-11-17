# Drive — personal GPS web app

A navigation page for iPhone: live position, route comparison by number of known
licence-plate cameras, and spoken alerts for plate cams, speed cams, traffic, roadwork,
closures, and your own police / damaged-road pins.

## Phone setup (about 5 minutes, repeat on each phone)
1. Open the site link in **Safari**.
2. Share button → **Add to Home Screen**. Open it from the home-screen icon from now on.
3. Tap **Start drive** and allow location ("Allow While Using App").
4. Tap the gear → paste your **TomTom API key** → Save. The key stays on that phone only.
5. iPhone Settings → Display & Brightness → Auto-Lock → **Never** while driving (the page also
   asks the phone to stay awake, but this is the reliable fallback).

The page must stay **open and on screen**. iPhones stop giving location to a web page in the
background.

## Using it
- **Where to?** Type an address or place. Or long-press the map → Route here.
- You get up to three cards: **Fastest**, **Fewer cams**, **Zero cams**, each with time,
  distance and known camera count. Tap a card to see it, then **Go**.
- If no camera-free route exists, the page says so and shows the lowest count it found.
  Cameras within ~200 m of your start or destination can't be avoided, so they're counted but not routed around.
- **Police / Road buttons** drop a pin where you are. You'll be warned at that spot next time.
  Tap a pin on the map to remove it. Pins live on your phone only.
- Speed bubble turns red over the limit + your margin (Settings).

## Optional: "Radios Off" shortcut
A web page cannot switch Bluetooth/Wi-Fi itself. To get a one-tap button:
1. Shortcuts app → **+** → Add Action → **Set Bluetooth** → Off.
2. Add Action → **Set Wi-Fi** → Off.
3. Name it **Radios Off** (or match the name in the app's Settings).
4. In the app: gear → **Run that Shortcut now**.
Wireless CarPlay needs both radios on, so this only suits a cabled connection.

## Data sources and limits
- Map, plate cameras (DeFlock mappers) and speed cameras: OpenStreetMap, crowdsourced.
  "0 known cams" is not a guarantee.
- Routing: the free public Valhalla server (valhalla1.openstreetmap.de), best effort, no traffic in routing.
- Traffic incidents, speed limits, search: TomTom (your key). Without a key, search falls back to
  OpenStreetMap and traffic / speed limits are off.

## Testing on a computer
Serve the folder (`python -m http.server`) and open `index.html?sim=LAT,LON`. Pressing Go
then drives the route automatically at about 135 mph so alerts can be checked.
