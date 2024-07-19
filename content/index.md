---
aliases: start
tags: 
  - index
  - start
  - BBZ
title: Testseite
draft: true
---

## Verlinkung

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

%%group by file.folder%%

```dataview
TABLE without ID
file.link AS "<span style='color:green'>Erwünschte Datei(en)"
WHERE row["draft"] != false
WHERE (contains(tags, "account") OR contains(tags, "privat") OR contains(tags, "Passwort") OR contains(tags, "password"))

```

```query
(Passwort OR password OR Zugangsdaten OR account OR login OR Benutzer OR Login) -file:index
```
%%

[[BBZ]]
[[index]]

%%
## Alle Dateien
```query
-tag:none
```
%%

[[Formatierungen]]

![[Sommer.png]]

