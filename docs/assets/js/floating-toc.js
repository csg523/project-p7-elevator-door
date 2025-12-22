const banner = document.querySelector(".project-banner");
if (banner) {
  document.documentElement.style.setProperty(
    "--banner-height",
    `${banner.offsetHeight}px`
  );
}

document.addEventListener("DOMContentLoaded", () => {
  const toc = document.getElementById("floating-toc");
  const tocList = document.getElementById("toc-list");
  const expandAllBtn = document.getElementById("toc-expand-all");
  const collapseAllBtn = document.getElementById("toc-collapse-all");
  const mobileToggleBtn = document.getElementById("toc-toggle-mobile");
  const mobileButton = document.getElementById("mobile-toc-button");

  if (!toc || !tocList) return;

  function slugify(text) {
    return text.toLowerCase().trim()
      .replace(/[^a-z0-9]+/g, "-")
      .replace(/(^-|-$)/g, "");
  }

  /* ===== Build TOC ===== */

  const sectionMap = [];

  document.querySelectorAll("section.page-section").forEach(section => {
    const sectionHeading = section.querySelector(".section-heading");
    if (!sectionHeading) return;

    const sectionTitle = sectionHeading.textContent;
    const sectionId = section.id || slugify(sectionTitle);
    section.id = sectionId;

    const li = document.createElement("li");
    li.className = "toc-section";

    const a = document.createElement("a");
    a.href = `#${sectionId}`;
    a.textContent = sectionTitle;
    a.className = "toc-h1";

    li.appendChild(a);

    const subList = document.createElement("ul");

    section.querySelectorAll(".section-content h1, .section-content h2, .section-content h3")
      .forEach(h => {
        if (!h.id) {
          h.id = slugify(sectionId + "-" + h.textContent);
        }

        const subLi = document.createElement("li");
        subLi.className = `toc-${h.tagName.toLowerCase()}`;

        const subA = document.createElement("a");
        subA.href = `#${h.id}`;
        subA.textContent = h.textContent;

        subLi.appendChild(subA);
        subList.appendChild(subLi);
      });

    if (subList.children.length > 0) {
      li.appendChild(subList);
    }

    tocList.appendChild(li);
    sectionMap.push({ section, li });
  });

  /* ===== Expand / collapse ===== */

  expandAllBtn.onclick = () => {
    document.querySelectorAll(".toc-section").forEach(li => li.classList.add("expanded"));
  };

  collapseAllBtn.onclick = () => {
    document.querySelectorAll(".toc-section").forEach(li => li.classList.remove("expanded"));
  };

  tocList.addEventListener("click", e => {
    if (e.target.classList.contains("toc-h1")) {
      e.target.parentElement.classList.toggle("expanded");
    }
  });

  /* ===== Mobile toggle ===== */

  if (mobileButton) {
    mobileButton.addEventListener("click", () => {
        toc.classList.toggle("mobile-visible");
    });
  }

  /* ===== Scroll spy ===== */

  const headings = Array.from(document.querySelectorAll(
    "section.page-section, .section-content h1, .section-content h2, .section-content h3"
  ));

  function onScroll() {
    let current = null;

    for (const h of headings) {
      if (h.getBoundingClientRect().top <= 100) {
        current = h;
      }
    }

    if (!current) return;

    // Clear active states
    document.querySelectorAll(".floating-toc a").forEach(a => a.classList.remove("active"));

    const activeLink = document.querySelector(`.floating-toc a[href="#${current.id}"]`);
    if (activeLink) {
      activeLink.classList.add("active");

      // Auto-expand parent section
      const sectionLi = activeLink.closest(".toc-section");
      if (sectionLi) sectionLi.classList.add("expanded");
    }

    if (window.innerWidth < 992) {
       toc.classList.remove("mobile-visible");
    }
  }

  document.addEventListener("scroll", onScroll);
  onScroll();
});
