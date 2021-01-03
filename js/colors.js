// Generated by CoffeeScript 2.5.1
(function() {
  this.COLORS = {
    // GitHub colors
    '1C Enterprise': '#814CCC',
    'ABAP': '#E8274B',
    'AGS Script': '#B9D9FF',
    'AL': '#3AA2B5',
    'AMPL': '#E6EFBB',
    'ANTLR': '#9DC3FF',
    'API Blueprint': '#2ACCA8',
    'APL': '#5A8164',
    'ASP.NET': '#9400FF',
    'ATS': '#1AC620',
    'ActionScript': '#882B0F',
    'Ada': '#02F88C',
    'Agda': '#315665',
    'Alloy': '#64C800',
    'AngelScript': '#C7D7DC',
    'Apex': '#1797C0',
    'Apollo Guidance Computer': '#0B3D91',
    'AppleScript': '#101F1F',
    'Arc': '#AA2AFE',
    'AspectJ': '#A957B0',
    'Assembly': '#6E4C13',
    'Asymptote': '#FF0000',
    'AutoHotkey': '#6594B9',
    'AutoIt': '#1C3552',
    'Ballerina': '#FF5000',
    'Batchfile': '#C1F12E',
    'Bison': '#6A463F',
    'Blade': '#F7523F',
    'BlitzMax': '#CD6400',
    'Boo': '#D4BEC1',
    'Brainfuck': '#2F2530',
    'Browserslist': '#FFD539',
    'C': '#555555',
    'C#': '#178600',
    'C++': '#F34B7D',
    'CSON': '#244776',
    'CSS': '#563D7C',
    'Ceylon': '#DFA535',
    'Chapel': '#8DC63F',
    'Cirru': '#CCCCFF',
    'Clarion': '#DB901E',
    'Classic ASP': '#6A40FD',
    'Clean': '#3F85AF',
    'Click': '#E4E6F3',
    'Clojure': '#DB5855',
    'CoffeeScript': '#244776',
    'ColdFusion': '#ED2CD6',
    'ColdFusion CFC': '#ED2CD6',
    'Common Lisp': '#3FB68B',
    'Common Workflow Language': '#B5314C',
    'Component Pascal': '#B0CE4E',
    'Crystal': '#000100',
    'Cuda': '#3A4E3A',
    'D': '#BA595E',
    'DM': '#447265',
    'Dafny': '#FFEC25',
    'Dart': '#00B4AB',
    'DataWeave': '#003A52',
    'Dhall': '#DFAFFF',
    'Dockerfile': '#384D54',
    'Dogescript': '#CCA760',
    'Dylan': '#6C616E',
    'E': '#CCCE35',
    'ECL': '#8A1267',
    'EJS': '#A91E50',
    'EQ': '#A78649',
    'Eiffel': '#4D6977',
    'Elixir': '#6E4A7E',
    'Elm': '#60B5CC',
    'Emacs Lisp': '#C065DB',
    'EmberScript': '#FFF4F3',
    'Erlang': '#B83998',
    'F#': '#B845FC',
    'F*': '#572E30',
    'FLUX': '#88CCFF',
    'Factor': '#636746',
    'Fancy': '#7B9DB4',
    'Fantom': '#14253C',
    'Faust': '#C37240',
    'Forth': '#341708',
    'Fortran': '#4D41B1',
    'FreeMarker': '#0050B2',
    'Frege': '#00CAFE',
    'Futhark': '#5F021F',
    'G-code': '#D08CF2',
    'GAML': '#FFC766',
    'GDScript': '#355570',
    'Game Maker Language': '#71B417',
    'Genie': '#FB855D',
    'Gherkin': '#5B2063',
    'Glyph': '#C1AC7F',
    'Gnuplot': '#F0A9F0',
    'Go': '#00ADD8',
    'Golo': '#88562A',
    'Gosu': '#82937F',
    'Grammatical Framework': '#FF0000',
    'GraphQL': '#E10098',
    'Groovy': '#E69F56',
    'HTML': '#E34C26',
    'Hack': '#878787',
    'Haml': '#ECE2A9',
    'Handlebars': '#F7931E',
    'Harbour': '#0E60E3',
    'Haskell': '#5E5086',
    'Haxe': '#DF7900',
    'HiveQL': '#DCE200',
    'HolyC': '#FFEFAF',
    'Hy': '#7790B2',
    'IDL': '#A3522F',
    'IGOR Pro': '#0000CC',
    'Idris': '#B30000',
    'Io': '#A9188D',
    'Ioke': '#078193',
    'Isabelle': '#FEFE00',
    'J': '#9EEDFF',
    'JFlex': '#DBCA00',
    'JSONiq': '#40D47E',
    'Java': '#B07219',
    'JavaScript': '#F1E05A',
    'Jolie': '#843179',
    'Jsonnet': '#0064BD',
    'Julia': '#A270BA',
    'Jupyter Notebook': '#DA5B0B',
    'KRL': '#28430A',
    'Kaitai Struct': '#773B37',
    'Kotlin': '#F18E33',
    'LFE': '#4C3023',
    'LLVM': '#185619',
    'LOLCODE': '#CC9900',
    'LSL': '#3D9970',
    'Lark': '#0B130F',
    'Lasso': '#999999',
    'Latte': '#F2A542',
    'Less': '#1D365D',
    'Lex': '#DBCA00',
    'LiveScript': '#499886',
    'LookML': '#652B81',
    'Lua': '#000080',
    'MATLAB': '#E16737',
    'MAXScript': '#00A6A6',
    'MLIR': '#5EC8DB',
    'MQL4': '#62A8D6',
    'MQL5': '#4A76B8',
    'MTML': '#B7E1F4',
    'Macaulay2': '#D8FFFF',
    'Makefile': '#427819',
    'Markdown': '#083FA1',
    'Marko': '#42BFF2',
    'Mask': '#F97732',
    'Max': '#C4A79C',
    'Mercury': '#FF2B2B',
    'Meson': '#007800',
    'Metal': '#8F14E9',
    'Mirah': '#C7A938',
    'Modula-3': '#223388',
    'NCL': '#28431F',
    'NWScript': '#111522',
    'Nearley': '#990000',
    'Nemerle': '#3D3C6E',
    'NetLinx': '#0AA0FF',
    'NetLinx+ERB': '#747FAA',
    'NetLogo': '#FF6375',
    'NewLisp': '#87AED7',
    'Nextflow': '#3AC486',
    'Nim': '#FFC200',
    'Nit': '#009917',
    'Nix': '#7E7EFF',
    'Nu': '#C9DF40',
    'NumPy': '#9C8AF9',
    'OCaml': '#3BE133',
    'ObjectScript': '#424893',
    'Objective-C': '#438EFF',
    'Objective-C++': '#6866FB',
    'Objective-J': '#FF0C5A',
    'Odin': '#60AFFE',
    'Omgrofl': '#CABBFF',
    'Opal': '#F7EDE0',
    'OpenQASM': '#AA70FF',
    'Oxygene': '#CDD0E3',
    'Oz': '#FAB738',
    'P4': '#7055B5',
    'PHP': '#4F5D95',
    'PLSQL': '#DAD8D8',
    'Pan': '#CC0000',
    'Papyrus': '#6600CC',
    'Parrot': '#F3CA0A',
    'Pascal': '#E3F171',
    'Pawn': '#DBB284',
    'Pep8': '#C76F5B',
    'Perl': '#0298C3',
    'PigLatin': '#FCD7DE',
    'Pike': '#005390',
    'PogoScript': '#D80074',
    'PostScript': '#DA291C',
    'PowerBuilder': '#8F0F8D',
    'PowerShell': '#012456',
    'Prisma': '#0C344B',
    'Processing': '#0096D8',
    'Prolog': '#74283C',
    'Propeller Spin': '#7FA2A7',
    'Pug': '#A86454',
    'Puppet': '#302B6D',
    'PureBasic': '#5A6986',
    'PureScript': '#1D222D',
    'Python': '#3572A5',
    'Q#': '#FED659',
    'QML': '#44A51C',
    'Qt Script': '#00B841',
    'Quake': '#882233',
    'R': '#198CE7',
    'RAML': '#77D9FB',
    'RUNOFF': '#665A4E',
    'Racket': '#3C5CAA',
    'Ragel': '#9D5200',
    'Raku': '#0000FB',
    'Rascal': '#FFFAA0',
    'ReScript': '#ED5051',
    'Reason': '#FF5847',
    'Rebol': '#358A5B',
    'Red': '#F50000',
    "Ren'Py": '#FF7F7F',
    'Ring': '#2D54CB',
    'Riot': '#A71E49',
    'Roff': '#ECDEBE',
    'Rouge': '#CC0088',
    'Ruby': '#701516',
    'Rust': '#DEA584',
    'SAS': '#B34936',
    'SCSS': '#C6538C',
    'SQF': '#3F3F3F',
    'SRecode Template': '#348A34',
    'SVG': '#FF9900',
    'SaltStack': '#646464',
    'Sass': '#A53B70',
    'Scala': '#C22D40',
    'Scheme': '#1E4AEC',
    'Self': '#0579AA',
    'Shell': '#89E051',
    'Shen': '#120F14',
    'Slash': '#007EFF',
    'Slice': '#003FA2',
    'Slim': '#2B2B2B',
    'SmPL': '#C94949',
    'Smalltalk': '#596706',
    'Solidity': '#AA6746',
    'SourcePawn': '#F69E1D',
    'Squirrel': '#800000',
    'Stan': '#B2011D',
    'Standard ML': '#DC566D',
    'Starlark': '#76D275',
    'Stylus': '#FF6347',
    'SuperCollider': '#46390B',
    'Svelte': '#FF3E00',
    'Swift': '#FFAC45',
    'SystemVerilog': '#DAE1C2',
    'TI Program': '#A0AA87',
    'Tcl': '#E4CC98',
    'TeX': '#3D6117',
    'Terra': '#00004C',
    'Turing': '#CF142B',
    'Twig': '#C1D026',
    'TypeScript': '#2B7489',
    'Unified Parallel C': '#4E3617',
    'Uno': '#9933CC',
    'UnrealScript': '#A54C4D',
    'V': '#4F87C4',
    'VBA': '#867DB1',
    'VBScript': '#15DCDC',
    'VCL': '#148AA8',
    'VHDL': '#ADB2CB',
    'Vala': '#FBE5CD',
    'Verilog': '#B2B7F8',
    'Vim script': '#199F4B',
    'Visual Basic .NET': '#945DB7',
    'Volt': '#1F1F1F',
    'Vue': '#2C3E50',
    'WebAssembly': '#04133B',
    'Wollok': '#A23738',
    'X10': '#4B6BEF',
    'XC': '#99DA07',
    'XQuery': '#5232E7',
    'XSLT': '#EB8CEB',
    'YAML': '#CB171E',
    'YARA': '#220000',
    'YASnippet': '#32AB90',
    'Yacc': '#4B6C4B',
    'ZAP': '#0D665E',
    'ZIL': '#DC75E5',
    'ZenScript': '#00BCD1',
    'Zephir': '#118F9E',
    'Zig': '#EC915C',
    'eC': '#913960',
    'mIRC Script': '#3D57C3',
    'mcfunction': '#E22837',
    'nesC': '#94B0C7',
    'ooc': '#B0B77E',
    'q': '#0040CD',
    'sed': '#64B970',
    'wdl': '#42F1F4',
    'wisp': '#7582D1',
    'xBase': '#403A40',
    // Custom colors
    'Machine Code': '#000000',
    'Java Bytecode': '#222222',
    'CIL': '#222222',
    'LLVM IR': '#185619'
  };

}).call(this);

//# sourceMappingURL=colors.js.map
