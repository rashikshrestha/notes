---
aliases: start
tags: 
  - index
  - start
  - BBZ
title:
draft: false
---

## Dies ist die index.md

> Alle Dateien, die auf dieser Website *sichtbar* sein sollen müssen 
> 
> - die Eigenschaft `draft: false` und 
> - dürfen nicht `draft:` (!) 
> 
> - im YAML front matter stehen haben!

Von Hand gesetzt: 2024-07-19 13:02 Uhr

%%
`= this.file.mtime`

Erstellungsdatum: `$= dv.current().file.ctime`

Änderungsdatum: `$= dv.current().file.mtime`

> [!NOTE]
> 
> > Wichtig!
> > Die folgenden Abfragen ==müssen== immer leer sein!

<https://quartz.jzhao.xyz/plugins/RemoveDrafts>

```dataview
TABLE without ID
file.link AS "<span style='color:red'>Unerwünschte Datei(en)"
WHERE row["draft"] != true
WHERE (contains(tags, "account") OR contains(tags, "privat") OR contains(tags, "Passwort") OR contains(tags, "password"))

```

```dataview
TABLE without ID
file.link AS "<span style='color:green'>Erwünschte Datei(en)"
WHERE row["draft"] != false
WHERE (contains(tags, "account") OR contains(tags, "privat") OR contains(tags, "Passwort") OR contains(tags, "password"))

```

```query
(Passwort OR password OR Zugangsdaten OR account OR login OR Benutzer OR Login) -file:index
```

## Alle Dateien
```query
-tag:none
```

%%

[[BBZ]]

[[index]]

[[Formatierungen]]

![[Sommer.png]]

