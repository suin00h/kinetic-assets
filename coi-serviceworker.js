/* cross-origin isolation을 서비스워커로 확보한다.
 *
 * GitHub Pages 같은 정적 호스팅은 응답 헤더를 못 바꾼다. COOP/COEP가 없으면
 * SharedArrayBuffer가 비활성이고 onnxruntime-web의 WASM 백엔드가 아무 말 없이
 * 단일 스레드로 떨어진다 — 느려지되 에러는 안 난다는 게 고약한 점이다.
 * (WebGPU 경로는 SharedArrayBuffer를 안 쓰므로 영향이 없다. 이건 폴백 보험이다.)
 *
 * 서비스워커가 모든 응답을 가로채 헤더를 붙여 돌려주면 브라우저는 isolated로 인정한다.
 * 처음 한 번은 워커가 제어권을 잡도록 새로고침이 필요하다.
 *
 * COEP는 require-corp를 쓴다. Safari는 credentialless를 구현하지 않아서
 * credentialless로 두면 조용히 격리에서 빠진다. 대신 CDN 스크립트를 전부
 * crossorigin="anonymous"로 불러 CORS 모드로 만족시킨다.
 */

if (typeof window === 'undefined') {
  // ── 서비스워커 컨텍스트 ──
  self.addEventListener('install', () => self.skipWaiting());
  self.addEventListener('activate', (e) => e.waitUntil(self.clients.claim()));

  self.addEventListener('fetch', (e) => {
    const req = e.request;
    if (req.cache === 'only-if-cached' && req.mode !== 'same-origin') return;

    e.respondWith(
      fetch(req)
        .then((res) => {
          if (res.status === 0) return res;          // opaque — 손대면 깨진다
          const h = new Headers(res.headers);
          h.set('Cross-Origin-Embedder-Policy', 'require-corp');
          h.set('Cross-Origin-Opener-Policy', 'same-origin');
          h.set('Cross-Origin-Resource-Policy', 'cross-origin');
          return new Response(res.body, { status: res.status, statusText: res.statusText, headers: h });
        })
        .catch((err) => console.error('coi-sw:', err))
    );
  });
} else {
  // ── 페이지 컨텍스트 ──
  (() => {
    if (window.crossOriginIsolated) return;          // 서버가 이미 헤더를 주고 있다
    if (!window.isSecureContext) return;             // http:// 에서는 워커 등록 자체가 안 된다
    if (!navigator.serviceWorker) return;

    navigator.serviceWorker.register(document.currentScript.src).then(
      (reg) => {
        reg.addEventListener('updatefound', () => window.location.reload());
        // 이미 활성 워커가 있는데 우리를 제어하지 않는다면 한 번 새로고침해야 잡힌다
        if (reg.active && !navigator.serviceWorker.controller) window.location.reload();
      },
      (err) => console.error('coi-sw 등록 실패:', err)
    );
  })();
}
