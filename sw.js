const CACHE_NAME = 'activa-key-v1';
const ASSETS = [
  './',
  './index.html',
  './manifest.json',
  './activa.png',
  './icon.png',
  './start.mp3',
  './click.mp3'
];

// Install Event
self.addEventListener('install', (e) => {
  e.waitUntil(
    caches.open(CACHE_NAME).then((cache) => {
      return cache.addAll(ASSETS);
    })
  );
});

// Fetch Event (Offline Support)
self.addEventListener('fetch', (e) => {
  e.respondWith(
    caches.match(e.request).then((response) => {
      return response || fetch(e.request);
    })
  );
});