¿Cuál es el promedio de la distancia en 1 hora?

from(bucket: "mediciones")
  |> range(start: -1h)
  |> filter(fn: (r) => r._measurement == "mediciones")
  |> filter(fn: (r) => r.equipo == "E32" and r.proyecto == "P26" and r.nodo == "nodo1" and r._field == "distancia_cm")
  |> mean()

¿Cuántos valores fueron menores a 50 en la ultima hora?

from(bucket: "mediciones")
  |> range(start: -1h)
  |> filter(fn: (r) => r._measurement == "mediciones")
  |> filter(fn: (r) => r.equipo == "E32" and r.proyecto == "P26" and r.nodo == "nodo1" and r._field == "distancia_cm")
  |> filter(fn: (r) => r._value < 50)
  |> count()

¿Cuántas veces en 24 horas el sensor estuvo en estado "sin eco"?

from(bucket: "mediciones")
  |> range(start: -24h)
  |> filter(fn: (r) => r._measurement == "mediciones")
  |> filter(fn: (r) => r.equipo == "E32" and r.proyecto == "P26" and r.nodo == "nodo1" and r._field == "sin_eco")
  |> filter(fn: (r) => r._value == 1)
  |> count()