import sympy as sp

### These genereates C code if the inverse of m needed to computed every loop of filter. It doesn't though so this is just here just in case.

def generate_inverse():
    k1, k2, k3 = sp.symbols('k1 k2 k3', real=True)

    m_12 = -(3 - k1**2 + (2*k1 + k2 + k3)*(k1 + k2))
    m_13 = -(k1 + k2)
    m_14 = -(1 + (k1 + k2)*(k3 - k1))
    
    m_32 = -((2*k1 + k2 + k3)*(k1 + k2) + 1)
    m_33 = -(2*k1 + k2)
    
    m_42 = -(2*k1 + k2 + k3)
    m_43 = -(k1 + k2)*(k3 - k1)

    M = sp.Matrix([
        [0,    1,    0,    0   ],
        [m_12, 0,    m_32, m_42],
        [m_13, 0,    m_33, -1  ],
        [m_14, 0,    m_43, -k3 ]
    ])

    # M^-1 = (1/det) * adjugate
    det = sp.simplify(M.det())
    adj = sp.simplify(M.adjugate())

    # --- Print C++ Output ---
    print("// Determinant")
    print(f"double det = {sp.ccode(det)};")
    print("double invDet = 1.0 / det;")
    print("")
    print("// Adjugate Matrix (Unscaled Inverse)")
    
    rows, cols = adj.shape
    for r in range(rows):
        for c in range(cols):
            val = adj[r, c]
            if val != 0:
                print(f"m_inv({r}, {c}) = ({sp.ccode(val)}) * invDet;")
            else:
                print(f"m_inv({r}, {c}) = 0;")
        print("")

if __name__ == "__main__":
    generate_inverse()