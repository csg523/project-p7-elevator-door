document.addEventListener("DOMContentLoaded", function () {
  const blocks = document.querySelectorAll('pre > code.language-mermaid');
  blocks.forEach(block => {
    const pre = block.parentElement;
    const div = document.createElement("div");
    div.className = "mermaid";
    div.textContent = block.textContent;
    pre.replaceWith(div);
  });
  if (document.querySelector('.mermaid')) {
    mermaid.initialize({
      startOnLoad: true,
      theme: "default",
      flowchart: { useMaxWidth: true },
      securityLevel: "loose"
    });
  }
});