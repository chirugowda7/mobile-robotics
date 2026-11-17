import json
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from collections import deque
from datetime import datetime

# ---------------- Store layout (grid cells, 1 cell ≈ 1.2 m) ----------------
COLS, ROWS, CELL = 24, 16, 20
ENTRANCE = (12, 15)
# name, colour, rectangle (x0, y0, x1, y1) with end exclusive
SECTIONS = [
    ("Electronics",    "#4a90d9", (3, 2, 5, 12)),
    ("Fashion",        "#9b59b6", (7, 2, 9, 12)),
    ("Home & Kitchen", "#e67e22", (11, 2, 13, 12)),
    ("Sports",         "#27ae60", (15, 2, 17, 12)),
    ("Grocery",        "#c0392b", (19, 2, 21, 12)),
]
COUNTER = (18, 13, 22, 15)
SECTION_RECT = {s[0]: s[2] for s in SECTIONS}
SECTION_COLOR = {s[0]: s[1] for s in SECTIONS}


def rect_cells(r):
    x0, y0, x1, y1 = r
    return {(x, y) for x in range(x0, x1) for y in range(y0, y1)}


BLOCKED = set().union(*(rect_cells(s[2]) for s in SECTIONS), rect_cells(COUNTER))

# ---------------- Product catalog (prices in INR) ----------------
CATALOG = [
    ("Wireless Headphones",       "Electronics",    1499, 4.3),
    ("Smartwatch Fit",            "Electronics",    2999, 4.1),
    ("USB-C Fast Charger 20W",    "Electronics",     799, 4.4),
    ("Wireless Mouse",            "Electronics",     599, 4.5),
    ("Bluetooth Speaker",         "Electronics",    1999, 4.2),
    ("Denim Jacket",              "Fashion",        1999, 4.0),
    ("Running Shoes",             "Fashion",        2499, 4.3),
    ("Cotton T-Shirt",            "Fashion",         499, 4.2),
    ("Woollen Cap",               "Fashion",         299, 3.9),
    ("Air Fryer 4L",              "Home & Kitchen", 3999, 4.4),
    ("Steel Water Bottle 1L",     "Home & Kitchen",  449, 4.6),
    ("Non-stick Frying Pan",      "Home & Kitchen",  899, 4.1),
    ("Mixer Grinder",             "Home & Kitchen", 2799, 4.2),
    ("Yoga Mat",                  "Sports",          799, 4.5),
    ("Adjustable Dumbbells 10kg", "Sports",         1899, 4.3),
    ("Cricket Bat",               "Sports",         1499, 4.0),
    ("Football Size 5",           "Sports",          699, 4.4),
    ("Basmati Rice 5kg",          "Grocery",         549, 4.6),
    ("Masala Chai 500g",          "Grocery",         320, 4.3),
    ("Almonds 500g",              "Grocery",         650, 4.7),
    ("Whole Wheat Atta 5kg",      "Grocery",         245, 4.2),
]
PRODUCTS = [{"name": n, "section": s, "price": p, "rating": r} for n, s, p, r in CATALOG]

FILLER = {"where", "is", "the", "find", "locate", "show", "me", "i", "want", "to", "buy",
          "a", "an", "for", "of", "do", "you", "have", "can", "how", "get", "go", "where's",
          "whats", "what's", "on", "any", "some", "please", "add", "in", "at", "it", "are",
          "shop", "looking"}
HELP = ("Try:\n"
        "• 'where is yoga mat' – shows the product's place and route on the map\n"
        "• 'add headphones' – adds the best match to your cart\n"
        "• 'cart' – cart total\n"
        "• 'sections' – list the sections in the mart")

BLUE, GREEN, RED, GRAY = "#1f6fb2", "#2e9e44", "#c0392b", "#6c6c6c"


def inr(amount):
    """Format a number as Indian rupees with lakh/crore grouping, e.g. ₹1,00,000."""
    s = str(int(round(amount)))
    if len(s) <= 3:
        return "₹" + s
    head, tail = s[:-3], s[-3:]
    parts = []
    while len(head) > 2:
        parts.insert(0, head[-2:])
        head = head[:-2]
    if head:
        parts.insert(0, head)
    return "₹" + ",".join(parts) + "," + tail


class SmartMart(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Smart Mart Assistant")
        self.configure(bg="#f0f0f0")

        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure("TFrame", background="#f0f0f0")
        style.configure("TLabel", background="#f0f0f0", font=("Segoe UI", 9))
        style.configure("TLabelframe", background="#f0f0f0")
        style.configure("TLabelframe.Label", background="#f0f0f0", font=("Segoe UI", 9, "bold"))

        self.cart = []
        self.results = []
        self.anim_job = None
        self.dist, self.parent = self.build_routes()

        self.build_ui()
        self.draw_map()
        self.refresh_products()
        self.post("Bot", "Hi! I'm your mart assistant. Ask me where a product is, "
                         "e.g. 'where are running shoes'. Type 'help' for more.")
        self.log("Mart assistant started")

    # ================= UI =================
    def build_ui(self):
        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=1)
        self.rowconfigure(1, weight=1)
        self.rowconfigure(2, weight=1)

        # Search & filters
        sf = ttk.LabelFrame(self, text="Search & Filters")
        sf.grid(row=0, column=0, columnspan=2, sticky="ew", padx=10, pady=(10, 5))
        ttk.Label(sf, text="Product:").grid(row=0, column=0, padx=6, pady=6, sticky="w")
        self.query_var = tk.StringVar()
        e = ttk.Entry(sf, textvariable=self.query_var, width=24)
        e.grid(row=0, column=1, sticky="we", padx=4)
        e.bind("<Return>", lambda ev: self.refresh_products())
        ttk.Label(sf, text="Section:").grid(row=0, column=2, padx=6, sticky="w")
        self.section_var = tk.StringVar(value="All")
        ttk.Combobox(sf, textvariable=self.section_var, state="readonly", width=14,
                     values=["All"] + [s[0] for s in SECTIONS]).grid(row=0, column=3, padx=4)
        self.btn(sf, "Search", GREEN, self.refresh_products).grid(row=0, column=4, padx=6)
        self.btn(sf, "Show All", GRAY, self.reset_search).grid(row=0, column=5, padx=(0, 6))

        ttk.Label(sf, text="Max Price").grid(row=1, column=0, padx=6, pady=4, sticky="w")
        self.price_var = tk.DoubleVar(value=5000)
        self.price_lbl = ttk.Label(sf, text="Up to ₹5,000", width=14)
        ttk.Scale(sf, from_=100, to=5000, variable=self.price_var, length=260,
                  command=lambda v: self.price_lbl.config(text=f"Up to {inr(float(v))}")
                  ).grid(row=1, column=1, sticky="we", padx=4)
        self.price_lbl.grid(row=1, column=2, padx=6, sticky="w")

        ttk.Label(sf, text="Min Rating").grid(row=2, column=0, padx=6, pady=4, sticky="w")
        self.rating_var = tk.DoubleVar(value=1.0)
        self.rating_lbl = ttk.Label(sf, text="1.0 ★", width=14)
        ttk.Scale(sf, from_=1, to=5, variable=self.rating_var, length=260,
                  command=lambda v: self.rating_lbl.config(text=f"{float(v):.1f} ★")
                  ).grid(row=2, column=1, sticky="we", padx=4)
        self.rating_lbl.grid(row=2, column=2, padx=6, sticky="w")
        sf.columnconfigure(1, weight=1)

        # Products list
        pf = ttk.LabelFrame(self, text="Products (₹ INR)")
        pf.grid(row=1, column=0, sticky="nsew", padx=(10, 5), pady=5)
        self.prod_tree = self.make_tree(pf, ("product", "section", "price", "rating"),
                                        ("Product", "Section", "Price", "Rating"),
                                        (190, 120, 80, 60))
        pb = ttk.Frame(pf)
        pb.pack(fill="x", padx=6, pady=(0, 6))
        self.btn(pb, "Locate on Map", BLUE, self.locate_selected).pack(side="left", padx=4)
        self.btn(pb, "Add to Cart", GREEN, self.add_selected).pack(side="left", padx=4)

        # Map
        mf = ttk.LabelFrame(self, text="Store Map & Path Indicator")
        mf.grid(row=1, column=1, sticky="nsew", padx=(5, 10), pady=5)
        self.canvas = tk.Canvas(mf, width=COLS * CELL, height=ROWS * CELL, bg="white",
                                highlightthickness=1, highlightbackground="#bbbbbb")
        self.canvas.pack(padx=6, pady=6)
        self.path_lbl = ttk.Label(mf, text="Select a product to see its route")
        self.path_lbl.pack(anchor="w", padx=8)
        ttk.Label(mf, text="● Entrance     ● Product spot     ━ Route (orange)"
                  ).pack(anchor="w", padx=8, pady=(2, 6))

        # Cart
        cf = ttk.LabelFrame(self, text="Shopping Cart")
        cf.grid(row=2, column=0, sticky="nsew", padx=(10, 5), pady=5)
        self.cart_tree = self.make_tree(cf, ("product", "price"), ("Product", "Price"),
                                        (300, 130))
        cb = ttk.Frame(cf)
        cb.pack(fill="x", padx=6, pady=6)
        self.total_lbl = ttk.Label(cb, text="Total: ₹0", font=("Segoe UI", 10, "bold"))
        self.total_lbl.pack(side="left", padx=4)
        self.btn(cb, "Remove", RED, self.remove_selected).pack(side="right", padx=3)
        self.btn(cb, "Clear", RED, self.clear_cart).pack(side="right", padx=3)
        self.btn(cb, "Load List", GRAY, self.load_cart).pack(side="right", padx=3)
        self.btn(cb, "Save List", GRAY, self.save_cart).pack(side="right", padx=3)

        # Assistant bot
        bf = ttk.LabelFrame(self, text="Assistant Bot")
        bf.grid(row=2, column=1, sticky="nsew", padx=(5, 10), pady=5)
        self.chat = tk.Text(bf, height=11, wrap="word", state="disabled", bg="white",
                            font=("Segoe UI", 9), relief="flat", padx=6, pady=6)
        self.chat.tag_configure("you", foreground=BLUE, font=("Segoe UI", 9, "bold"))
        self.chat.tag_configure("bot", foreground=GREEN, font=("Segoe UI", 9, "bold"))
        self.chat.pack(fill="both", expand=True, padx=6, pady=(6, 2))
        inp = ttk.Frame(bf)
        inp.pack(fill="x", padx=6, pady=(2, 4))
        self.chat_var = tk.StringVar()
        e2 = ttk.Entry(inp, textvariable=self.chat_var)
        e2.pack(side="left", fill="x", expand=True)
        e2.bind("<Return>", lambda ev: self.send_chat())
        self.btn(inp, "Send", BLUE, self.send_chat).pack(side="left", padx=4)
        quick = ttk.Frame(bf)
        quick.pack(fill="x", padx=6, pady=(0, 6))
        for label, cmd in (("Help", "help"), ("Cart", "cart"), ("Sections", "sections")):
            self.btn(quick, label, GRAY, lambda c=cmd: self.handle_chat(c)).pack(side="left", padx=3)

        # Log
        lf = ttk.LabelFrame(self, text="Log")
        lf.grid(row=3, column=0, columnspan=2, sticky="ew", padx=10, pady=(5, 10))
        self.log_box = tk.Text(lf, height=6, font=("Consolas", 9), bg="white",
                               wrap="word", state="disabled")
        sb = ttk.Scrollbar(lf, command=self.log_box.yview)
        self.log_box.configure(yscrollcommand=sb.set)
        self.log_box.pack(side="left", fill="both", expand=True, padx=(4, 0), pady=4)
        sb.pack(side="right", fill="y", pady=4)

    def btn(self, parent, text, color, cmd):
        return tk.Button(parent, text=text, command=cmd, bg=color, fg="white",
                         activebackground=color, activeforeground="white", relief="flat",
                         padx=10, pady=3, font=("Segoe UI", 9, "bold"), cursor="hand2")

    def make_tree(self, parent, cols, headings, widths):
        wrap = ttk.Frame(parent)
        wrap.pack(fill="both", expand=True, padx=6, pady=4)
        tree = ttk.Treeview(wrap, columns=cols, show="headings", height=6)
        for c, h, w in zip(cols, headings, widths):
            tree.heading(c, text=h)
            tree.column(c, width=w, anchor="w")
        sb = ttk.Scrollbar(wrap, orient="vertical", command=tree.yview)
        tree.configure(yscrollcommand=sb.set)
        tree.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")
        return tree

    def log(self, msg):
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_box.configure(state="normal")
        self.log_box.insert("end", f"[{ts}] {msg}\n")
        self.log_box.see("end")
        self.log_box.configure(state="disabled")

    def post(self, sender, text):
        self.chat.configure(state="normal")
        self.chat.insert("end", f"{sender}: ", "you" if sender == "You" else "bot")
        self.chat.insert("end", text + "\n\n")
        self.chat.configure(state="disabled")
        self.chat.see("end")

    # ================= Products =================
    def refresh_products(self, *_):
        q = self.query_var.get().strip().lower()
        section = self.section_var.get()
        max_price = self.price_var.get()
        min_rating = self.rating_var.get()
        self.results = [p for p in PRODUCTS
                        if (section == "All" or p["section"] == section)
                        and (not q or q in (p["name"] + " " + p["section"]).lower())
                        and p["price"] <= max_price and p["rating"] >= min_rating]
        self.results.sort(key=lambda p: p["price"])
        self.prod_tree.delete(*self.prod_tree.get_children())
        for i, p in enumerate(self.results):
            self.prod_tree.insert("", "end", iid=str(i), values=(
                p["name"], p["section"], inr(p["price"]), f"{p['rating']} ★"))
        self.log(f"Search '{q or 'all'}' in {section}: {len(self.results)} products")

    def reset_search(self):
        self.query_var.set("")
        self.section_var.set("All")
        self.price_var.set(5000)
        self.rating_var.set(1.0)
        self.price_lbl.config(text="Up to ₹5,000")
        self.rating_lbl.config(text="1.0 ★")
        self.refresh_products()

    def selected_product(self):
        sel = self.prod_tree.selection()
        if not sel:
            messagebox.showinfo("Products", "Select a product first.")
            return None
        return self.results[int(sel[0])]

    def locate_selected(self):
        p = self.selected_product()
        if p:
            self.locate_product(p)
            self.log(f"Located {p['name']} in {p['section']}")

    def add_selected(self):
        p = self.selected_product()
        if p:
            self.add_to_cart(p)

    # ================= Map & routing =================
    def build_routes(self):
        """Breadth-first search from the entrance over all free cells."""
        def free(x, y):
            return 0 <= x < COLS and 0 <= y < ROWS and (x, y) not in BLOCKED
        dist, parent = {ENTRANCE: 0}, {ENTRANCE: None}
        q = deque([ENTRANCE])
        while q:
            x, y = q.popleft()
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                n = (x + dx, y + dy)
                if free(*n) and n not in dist:
                    dist[n] = dist[(x, y)] + 1
                    parent[n] = (x, y)
                    q.append(n)
        return dist, parent

    def route_to(self, rect):
        """Shortest path from the entrance to an aisle cell next to the shelf."""
        x0, y0, x1, y1 = rect
        cand = ([(x, y0 - 1) for x in range(x0, x1)] + [(x, y1) for x in range(x0, x1)] +
                [(x0 - 1, y) for y in range(y0, y1)] + [(x1, y) for y in range(y0, y1)])
        reachable = [c for c in cand if c in self.dist]
        if not reachable:
            return []
        cur = min(reachable, key=lambda c: self.dist[c])
        path = []
        while cur is not None:
            path.append(cur)
            cur = self.parent[cur]
        return path[::-1]

    def center(self, cell):
        x, y = cell
        return x * CELL + CELL / 2, y * CELL + CELL / 2

    def draw_map(self):
        c = self.canvas
        c.delete("all")
        for x in range(COLS + 1):
            c.create_line(x * CELL, 0, x * CELL, ROWS * CELL, fill="#f0f0f0")
        for y in range(ROWS + 1):
            c.create_line(0, y * CELL, COLS * CELL, y * CELL, fill="#f0f0f0")
        for name, color, (x0, y0, x1, y1) in SECTIONS:
            c.create_rectangle(x0 * CELL, y0 * CELL, x1 * CELL, y1 * CELL,
                               fill=color, outline="")
            for y in range(y0 + 2, y1, 2):  # shelf bays
                c.create_line(x0 * CELL, y * CELL, x1 * CELL, y * CELL, fill="white")
            c.create_text(((x0 + x1) / 2) * CELL, (y0 - 1) * CELL + CELL / 2,
                          text=name, font=("Segoe UI", 7, "bold"), fill="#333")
        x0, y0, x1, y1 = COUNTER
        c.create_rectangle(x0 * CELL, y0 * CELL, x1 * CELL, y1 * CELL,
                           fill="#7f8c8d", outline="")
        c.create_text(((x0 + x1) / 2) * CELL, ((y0 + y1) / 2) * CELL, text="CHECKOUT",
                      font=("Segoe UI", 7, "bold"), fill="white")
        ex, ey = self.center(ENTRANCE)
        c.create_oval(ex - 7, ey - 7, ex + 7, ey + 7, fill=GREEN, outline="")
        c.create_text(ex, ey - 14, text="ENTRANCE", font=("Segoe UI", 7, "bold"), fill=GREEN)

    def locate_product(self, p):
        if self.anim_job:
            self.after_cancel(self.anim_job)
            self.anim_job = None
        path = self.route_to(SECTION_RECT[p["section"]])
        if not path:
            self.log("No route found to that section")
            return []
        self.draw_map()
        steps = len(path) - 1
        self.path_lbl.config(text=f"{p['name']} → {p['section']}  |  "
                                  f"{steps} steps (~{steps * 1.2:.0f} m)")
        self.draw_step(path, 1, p)
        return path

    def draw_step(self, path, i, p):
        """Animate the route one segment at a time, then mark the product spot."""
        if i < len(path):
            x1, y1 = self.center(path[i - 1])
            x2, y2 = self.center(path[i])
            self.canvas.create_line(x1, y1, x2, y2, fill="#e67e22", width=3,
                                    capstyle="round", tags="path")
            self.anim_job = self.after(40, lambda: self.draw_step(path, i + 1, p))
        else:
            self.anim_job = None
            tx, ty = self.center(path[-1])
            self.canvas.create_oval(tx - 8, ty - 8, tx + 8, ty + 8, fill="#e74c3c",
                                    outline="white", width=2)
            self.canvas.create_text(tx, ty - 16, text=p["name"][:22],
                                    font=("Segoe UI", 7, "bold"), fill="#e74c3c")

    # ================= Cart =================
    def add_to_cart(self, p):
        self.cart.append({"product": p["name"], "section": p["section"], "price": p["price"]})
        self.refresh_cart()
        self.log(f"Added {p['name']} ({inr(p['price'])})")

    def refresh_cart(self):
        self.cart_tree.delete(*self.cart_tree.get_children())
        for i, item in enumerate(self.cart):
            self.cart_tree.insert("", "end", iid=str(i),
                                  values=(item["product"], inr(item["price"])))
        total = sum(i["price"] for i in self.cart)
        self.total_lbl.config(text=f"Total: {inr(total)}  ({len(self.cart)} items)")

    def cart_summary(self):
        if not self.cart:
            return "Your cart is empty."
        total = sum(i["price"] for i in self.cart)
        return f"Cart: {len(self.cart)} item(s), total {inr(total)}."

    def remove_selected(self):
        sel = self.cart_tree.selection()
        for iid in sorted(sel, key=int, reverse=True):
            removed = self.cart.pop(int(iid))
            self.log(f"Removed {removed['product']}")
        self.refresh_cart()

    def clear_cart(self):
        if self.cart and messagebox.askyesno("Clear Cart", "Remove all items from the cart?"):
            self.cart.clear()
            self.refresh_cart()
            self.log("Cart cleared")

    def save_cart(self):
        if not self.cart:
            messagebox.showinfo("Save List", "Cart is empty.")
            return
        path = filedialog.asksaveasfilename(defaultextension=".json",
                                            filetypes=[("JSON", "*.json")])
        if path:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(self.cart, f, indent=2, ensure_ascii=False)
            self.log(f"Saved {len(self.cart)} items to {path}")

    def load_cart(self):
        path = filedialog.askopenfilename(filetypes=[("JSON", "*.json")])
        if path:
            with open(path, encoding="utf-8") as f:
                self.cart = json.load(f)
            self.refresh_cart()
            self.log(f"Loaded {len(self.cart)} items")

    # ================= Assistant bot =================
    def send_chat(self):
        text = self.chat_var.get()
        self.chat_var.set("")
        self.handle_chat(text)

    def handle_chat(self, text):
        text = text.strip()
        if not text:
            return
        self.post("You", text)
        words = [w.strip("?!.,'\"") for w in text.lower().split()]
        words = [w for w in words if w]
        if not words:
            return

        if words[0] in ("hi", "hello", "hey"):
            reply = "Hello! Tell me what you need, like 'where is yoga mat'. Type help to see more."
        elif "help" in words:
            reply = HELP
        elif "cart" in words or "total" in words:
            reply = self.cart_summary()
        elif "sections" in words or "categories" in words:
            reply = "We have: " + ", ".join(s[0] for s in SECTIONS) + "."
        else:
            add = "add" in words
            terms = [w for w in words if w not in FILLER]
            if not terms:
                reply = "Which product are you looking for? Example: 'where are running shoes'."
            else:
                matches = sorted(
                    (p for p in PRODUCTS
                     if all(t in (p["name"] + " " + p["section"]).lower() for t in terms)),
                    key=lambda p: p["price"])
                if not matches:
                    reply = (f"Sorry, I couldn't find anything matching '{' '.join(terms)}'. "
                             "Try a shorter name or type 'sections'.")
                elif add:
                    p = matches[0]
                    self.add_to_cart(p)
                    reply = f"Added {p['name']} ({inr(p['price'])}) to your cart. {self.cart_summary()}"
                else:
                    p = matches[0]
                    path = self.locate_product(p)
                    steps = max(len(path) - 1, 0)
                    reply = (f"{p['name']} is {inr(p['price'])} and is in the {p['section']} "
                             f"section. Follow the orange path on the map, about "
                             f"{steps * 1.2:.0f} m from the entrance.")
                    if len(matches) > 1:
                        reply += " Other matches: " + ", ".join(m["name"] for m in matches[1:4]) + "."
        self.post("Bot", reply)


if __name__ == "__main__":
    SmartMart().mainloop()