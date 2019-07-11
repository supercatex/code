import requests


class Crawler:

    @staticmethod
    def html(domain):
        req = requests.get(domain)
        return req.text

    @staticmethod
    def find(s, s_key, e_key='', idx=0):
        s_idx = s.find(s_key)
        i = 0
        while i < idx:
            m = s[s_idx+1:]
            n = m.find(s_key)
            if n > 0:
                s_idx += n + 1
            else:
                s_idx = -1
            i = i + 1
        if s_idx >= 0:
            s_idx += len(s_key)
            e_idx = -1
            if len(e_key) > 0:
                e_idx = s[s_idx:].find(e_key)
            if e_idx > 0:
                return s[s_idx:s_idx + e_idx]
            else:
                return s[s_idx:e_idx]
        return ''


if __name__ == '__main__':
    html = Crawler.html('https://www.google.com/?hl=zh-TW')
    print(html)

    path = Crawler.find(Crawler.find(html, '<img alt="Google"'), 'src="', '"')
    print(path)

    from PIL import Image
    import requests
    from io import BytesIO

    response = requests.get('https://www.google.com' + path)
    img = Image.open(BytesIO(response.content))
    img.show()
