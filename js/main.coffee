@LANGUAGES = new Set
elements = []

for compiler in COMPILERS
  {name, source, target, type} = compiler
  for language in [source, target]
    continue if LANGUAGES.has language
    elements.push
      data:
        id: language
        color: COLORS[language] or '#ccc'
    LANGUAGES.add language

  elements.push
    data:
      id: name
      source: source
      target: target
      type: type
      sourceColor: COLORS[source] or '#ccc'
      targetColor: COLORS[target] or '#ccc'

graph = null

window.onload = ->
  graph = cytoscape
    container: document.getElementById 'graph'
    elements: elements
    style: [
      selector: 'node'
      style:
        'label': 'data(id)'
        'font-size': (ele) -> Math.max(14, 9 + 0.25 * ele.incomers().length)
        'width': 'label'
        'height': (ele) -> Math.min(50, 10 + 2 * ele.incomers().length)
        'color': 'white'
        'background-color': 'data(color)'
        'text-valign': 'center'
        'padding-left': 10
        'padding-right': 10
        'padding-top': 10
        'padding-bottom': 10
    ,
      selector: 'edge'
      style:
        'label': 'data(id)'
        'width': 3
        'font-size': (ele) ->
          Math.min(Math.max(22 - ele.style()['label'].length, 9), 12)
        'color': 'black'
        'line-color': 'data(sourceColor)'
        'target-arrow-color': 'data(sourceColor)'
        'target-arrow-shape': 'triangle'
        'text-rotation': 'autorotate'
        'text-margin-y': -10
        'opacity': 0.7
        'curve-style': 'bezier'
    ]

  queryParams = {}
  for param in location.search.slice(1).split('&')
    [key, value] = param.split '='
    queryParams[key] = decodeURIComponent value

  {source, target, direct, cycles} = queryParams

  form = document.forms[0]
  [form.source.value, form.target.value,
   form.direct.checked, form.cycles.checked] =
    [source or '', target or '', direct, cycles]

  compile source, target, direct, cycles if source or target

  layout = graph.layout
    name: 'cose-bilkent'
    idealEdgeLength: 100
    nodeRepulsion: 1000000
    padding: 40
    randomize: false

  layout.run()

@filter = (e) ->
  e.preventDefault()
  form = e.currentTarget

  [source, target, direct, cycles] =
    [form.source.value, form.target.value,
     form.direct.checked, form.cycles.checked]

  updateURL { source, target, direct, cycles }
  compile source, target, direct, cycles

@show = ->
  updateURL()
  compile()

findPaths = (sourceNode, targetNode, path) ->
  path = sourceNode.cy().collection() if not path
  return yield path if sourceNode is targetNode
  for edge in sourceNode.outgoers 'edge'
    node = edge.target()
    continue if path.connectedNodes().contains node
    path.merge edge
    yield from findPaths node, targetNode, path
    path.unmerge edge

compile = (source, target, direct, cycles) ->
  info.innerText = 'Select a language from the list'

  return if (source and not LANGUAGES.has source) or
            (target and not LANGUAGES.has target)

  sourceNode = graph.getElementById(source) if source
  targetNode = graph.getElementById(target) if target

  if sourceNode and targetNode
    if direct or cycles
      elements = (
        if direct then sourceNode.edgesTo targetNode
        else sourceNode.successors().intersection targetNode.predecessors()
      ).add [sourceNode, targetNode]
      text = ''
    else
      elements = graph.collection().add [sourceNode, targetNode]
      count = 0
      for path from findPaths sourceNode, targetNode
        elements.merge path.connectedNodes()
        elements.merge path
        count += 1
      text =
        if count is 1 then "#{count} way to compile #{source} to #{target}"
        else "#{count} ways to compile #{source} to #{target}"
    showElements elements

  else if sourceNode
    elements = (
      if direct then sourceNode.outgoers() else sourceNode.successors()
    ).add sourceNode
    showElements elements

    count = elements.nodes().length - 1
    text =
      if count is 1 then "#{source} compiles to #{count} language"
      else "#{source} compiles to #{count} languages"
    text += " directly" if direct

  else if targetNode
    elements = (
      if direct then targetNode.incomers() else targetNode.predecessors()
    ).add targetNode
    showElements elements

    count = elements.nodes().length - 1
    text =
      if count is 1 then "#{count} language compiles to #{target}"
      else "#{count} languages compile to #{target}"
    text += " directly" if direct

  else
    elements = graph.elements()
    elements.style display: 'element'
    text = "#{LANGUAGES.size} languages"

  info.innerText = "#{text}\n#{elements.edges().length} compilers"

updateURL = (params) ->
  queryParts = []
  for key, value of params
    queryParts.push "#{key}=#{encodeURIComponent value}" if value

  history.replaceState params, '',
    if queryParts.length then "?#{queryParts.join '&'}" else location.pathname

showElements = (elements) ->
  graph.batch ->
    graph.elements().style display: 'none'
    elements.style display: 'element'
