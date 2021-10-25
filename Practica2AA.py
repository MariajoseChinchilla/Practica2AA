# hay que escribir un pseudo codigo para calcular la raiz k esima
# los numeros estan en el formato del KS chip, base 256

def kroot(num1, k, start,iterations):        #parecido al metodo de Newton
    x0 = start
    result = start
    for i in range(0,iterations):
        xn = x0 - ((x0**k - num1)/(k*x0**(k-1)))
        x0 = xn
        result = xn
    return result


