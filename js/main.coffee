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

defaultText = "#{LANGUAGES.size} languages\n#{COMPILERS.length} compilers"

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
        'text-opacity': 10
        'opacity': 0.7
        'curve-style': 'bezier'
    ]

  graph.layout
    name: 'cose-bilkent'
    idealEdgeLength: 100
    nodeRepulsion: 50000
    padding: 40
    random: false

@filter = (e) ->
  e.preventDefault()
  form = e.currentTarget

  source = form.source.value
  target = form.target.value

  return show() if not source and not target

  info.innerText = 'Select a language from the list'

  return if (source and not LANGUAGES.has source) or
            (target and not LANGUAGES.has target)

  sourceNode = graph.getElementById(source) if source
  targetNode = graph.getElementById(target) if target

  if sourceNode and targetNode
    elements = sourceNode.successors().intersection targetNode.predecessors()
                         .add [sourceNode, targetNode]
    showElements elements
    text = ''

  else if sourceNode
    elements = sourceNode.successors().add sourceNode
    showElements elements

    count = elements.filter('node').length - 1
    text =
      if count is 1 then "#{source} compiles to #{count} language"
      else "#{source} compiles to #{count} languages"

  else if targetNode
    elements = targetNode.predecessors().add targetNode
    showElements elements

    count = elements.filter('node').length - 1
    text =
      if count is 1 then "#{count} language compiles to #{target}"
      else "#{count} languages compile to #{target}"

  info.innerText = "#{text}\n#{elements.filter('edge').length} compilers"

showElements = (elements) ->
  graph.batch ->
    graph.elements().style display: 'none'
    elements.style display: 'element'

@show = (e) ->
  graph.elements().style display: 'element'
  info.innerText = defaultText
