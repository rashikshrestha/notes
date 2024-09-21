import { QuartzConfig } from "./quartz/cfg"
import * as Plugin from "./quartz/plugins"

/**
 * Quartz 4.0 Configuration
 *
 * See https://quartz.jzhao.xyz/configuration for more information.
 */
const config: QuartzConfig = {
  configuration: {
    pageTitle: "🤖 Rashik's Notes",
    enableSPA: true,
    enablePopovers: true,
    analytics: {
      provider: "plausible",
    },
    locale: "en-US",
    baseUrl: "rashik.info.np/notes",
    ignorePatterns: ["private", "templates", ".obsidian"],
    defaultDateType: "created",
    theme: {
      fontOrigin: "googleFonts",
      cdnCaching: true,
      typography: {
        header: "Schibsted Grotesk",
        body: "Source Sans Pro",
        code: "IBM Plex Mono",
      },
      colors: {
        lightMode: {
          light: "#faf8f8",
          lightgray: "#e5e5e5",
          gray: "#b8b8b8",
          darkgray: "#4e4e4e",
          dark: "#2b2b2b",
          secondary: "#284b63",
          tertiary: "#84a59d",
          highlight: "rgba(143, 159, 169, 0.15)",
          textHighlight: "#fff23688",
        },
        darkMode: {
          // Default
          // light: "#161618",
          // lightgray: "#393639",
          // gray: "#646464",
          // darkgray: "#d4d4d4",
          // dark: "#ebebec",
          // secondary: "#7b97aa",
          // tertiary: "#84a59d",
          // highlight: "rgba(143, 159, 169, 0.15)",
          // textHighlight: "#b3aa0288",

          //Obsidian Nord
          // light: "#2e3440",
          // lightgray: "#3b4252",
          // gray: "#4c566a",
          // darkgray: "#d8dee9",
          // dark: "#eceff4",
          // secondary: "#88c0d0",
          // tertiary: "#81a1c1",
          // highlight: "rgba(136, 192, 208, 0.15)",
          // textHighlight: "#b3aa0288",

          // JellyFish
          // light: "#0a0f1f",
          // lightgray: "#21334e",
          // gray: "#536b88",
          // darkgray: "#aac6ea",
          // dark: "#ffffff",
          // secondary: "#ff69b4",
          // tertiary: "#32c6ff",
          // highlight: "rgba(255, 105, 180, 0.15)",
          // textHighlight: "#b3aa0288",

          // Visual Studio
          light: "#1e1e1e",
          lightgray: "#2d2d2d",
          gray: "#8a8989",
          darkgray: "#d4d4d4",
          dark: "#ffffff",
          secondary: "#569cd6",
          tertiary: "#4ec9b0",
          highlight: "rgba(86, 156, 214, 0.15)",
          textHighlight: "#b3aa0288",

          // Dracula
          // light: "#282a36",
          // lightgray: "#44475a",
          // gray: "#6272a4",
          // darkgray: "#f8f8f2",
          // dark: "#bd93f9",
          // secondary: "#ff79c6",
          // tertiary: "#8be9fd",
          // highlight: "rgba(255, 121, 198, 0.15)"
          // textHighlight: "#b3aa0288",
        },
      },
    },
  },
  plugins: {
    transformers: [
      Plugin.FrontMatter(),
      Plugin.CreatedModifiedDate({
        priority: ["frontmatter", "filesystem"],
      }),
      Plugin.Latex({ renderEngine: "katex" }),
      Plugin.SyntaxHighlighting({
        theme: {
          light: "github-light",
          dark: "github-dark",
        },
        keepBackground: false,
      }),
      Plugin.ObsidianFlavoredMarkdown({ enableInHtmlEmbed: false }),
      Plugin.GitHubFlavoredMarkdown(),
      Plugin.TableOfContents(),
      Plugin.CrawlLinks({ markdownLinkResolution: "shortest" }),
      Plugin.Description(),
    ],
    filters: [Plugin.RemoveDrafts()],
    emitters: [
      Plugin.AliasRedirects(),
      Plugin.ComponentResources(),
      Plugin.ContentPage(),
      Plugin.FolderPage(),
      Plugin.TagPage(),
      Plugin.ContentIndex({
        enableSiteMap: true,
        enableRSS: true,
      }),
      Plugin.Assets(),
      Plugin.Static(),
      Plugin.NotFoundPage(),
    ],
  },
}

export default config
